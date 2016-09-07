
//
// Copyright 2010-2011,2014 Ettus Research LLC
// Modified 2016 by Max Li
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
 
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <iostream>
#include <csignal>
#include <complex>
#include <pthread.h>
#include <alsa/asoundlib.h>
#define PI 3.14159265359
#define INV2PI 0.15915494309
#define START_PLAYBACK 1200 //Threshold to start ALSA playback
const float coeff_1 = PI / 4, coeff_2 = 3 * coeff_1; //For Atan2, Pi/4 and 3Pi/4
typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

//Define a struct to pass into the recv thread 
struct proc_recv{
	uhd::rx_streamer::sptr recv_stream;
	std::complex<float> *buff;
	uhd::rx_metadata_t md;
	int size;
};

//Audio Handles
snd_pcm_t *playback_handle;
int err;
//FIR coefficients, filter 640kHz to 32 kHz
#define BL 21
const float B[21] = {
   0.006768081803,  0.01703602448,  0.02830111235,  0.04008036107,  0.05183677003,
    0.06301041692,  0.07305181772,  0.08145540953,  0.08779074997,  0.09172940999,
    0.09306564927,  0.09172940999,  0.08779074997,  0.08145540953,  0.07305181772,
    0.06301041692,  0.05183677003,  0.04008036107,  0.02830111235,  0.01703602448,
   0.006768081803
};

//Function Prototypes
template<typename samp_type> void recv_to_file(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &cpu_format,
    const std::string &wire_format,
    size_t samps_per_buff,
    float gain
);
void *recvTask(void* ptr);
bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time);
float arctan2(float y, float x);
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
 

 /* MAIN,INITIALIZATION */ 
 
int UHD_SAFE_MAIN(int argc, char *argv[]){
	uhd::set_thread_priority_safe(); 
    
	
	namespace po = boost::program_options;
	
    std::string args,type, ant, subdev, ref, wirefmt;
    size_t spb;
    //size_t total_num_samps;
    float vgain;
    double rate, freq, gain, bw, setup_time;
    int i;
    
	////////////////////////////////////////////////
	//CLI Options
	po::options_description desc("Allowed options");
	desc.add_options()
	        ("help", "help message")
	        ("freq", po::value<double>(&freq)->default_value(107.3), "RF center frequency in MHz")
	        ("vgain", po::value<float>(&vgain)->default_value(1), "volume gain")
	    ;
	
	    po::variables_map vm;
	    po::store(po::parse_command_line(argc, argv, desc), vm);
	    po::notify(vm);

	    //print the help message
	    if (vm.count("help")) {
	        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
	        std::cout
	            << std::endl
	            << "This application streams data from a single channel of a USRP device to a file.\n"
	            << std::endl;
	        return ~0;
	    }
	    
	/////////////////////////////////////////////////////
    ////////Configure radio    


    /*PUT RADIO PARAMETERS HERE*/
    args = "";
    type = "float";
    spb = 10000; //Numbers of samples in a buffer
    rate = 640000; //Cannot = 0
    freq = freq*1e6;
    //freq = 96100000;
    gain = 40;
    ant  = "RX2";  //ant can be "RX/TX" or "RX2"
    subdev = "A:A"; //subdev can be "A:A" or "A:B"
    bw = 0; //don't set bw
    ref = "internal"; //internal, external, mimo
    wirefmt = "sc16"; //or sc8
    setup_time = 1.0; //sec setup
     
    //Create USRP object
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
 
    //Lock mboard clocks
    usrp->set_clock_source(ref);
     
    if(subdev != "NULL") usrp->set_rx_subdev_spec(subdev);
 
    //set rx rate
    usrp->set_rx_rate(rate);
    //Set rx freq
    uhd::tune_request_t tune_request(freq);
    usrp->set_rx_freq(tune_request);
    //Set gain
    usrp->set_rx_gain(gain);
    //Set front end analog bandwidth
    if (bw) usrp->set_rx_bandwidth(bw);
    //set the antenna
    if (ant != "NULL") usrp->set_rx_antenna(ant);
    //allow for some setup time
    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); 
    //check Ref and LO Lock detect
	check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, 0), setup_time);
	if (ref == "mimo")
		check_locked_sensor(usrp->get_mboard_sensor_names(0), "mimo_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);
	if (ref == "external")
		check_locked_sensor(usrp->get_mboard_sensor_names(0), "ref_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);


    ////////Configure ALSA
    
    unsigned int rate_audio = 32000;
    snd_pcm_uframes_t buffer_size = 8192;
    
    //DEBUG AUDIO
    snd_output_t* out;
    snd_output_stdio_attach(&out, stderr, 0);
    
    //Audio Parameter handles
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_hw_params_t *hw_params;
    
    //Hardware parameters: (Playback, interleaved, LE float, rate, channels);
	err = snd_pcm_open(&playback_handle,"default",SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK); //Set in non-blocking mode, so audio samples get dropped.
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
    err = snd_pcm_hw_params_malloc (&hw_params);		
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_any (playback_handle, hw_params);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_set_access (playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_set_format (playback_handle, hw_params, SND_PCM_FORMAT_FLOAT_LE);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_set_buffer_size_near (playback_handle, hw_params, &buffer_size);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_set_rate_near (playback_handle, hw_params, &rate_audio, (int*)0);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params_set_channels (playback_handle, hw_params, 2); //Change to 2 for LR output.
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	err = snd_pcm_hw_params (playback_handle, hw_params);
	if (err < 0) std::cerr <<  snd_strerror(err) <<std::endl;
	snd_pcm_hw_params_free (hw_params);
	//Software Parameters: (start threshold)
    snd_pcm_sw_params_malloc (&sw_params);
    snd_pcm_sw_params_current (playback_handle, sw_params);   
    snd_pcm_sw_params_set_start_threshold(playback_handle, sw_params, START_PLAYBACK);
    snd_pcm_sw_params(playback_handle, sw_params);
    snd_pcm_sw_params_free(sw_params);
    //Print sw parameter setup.
    snd_pcm_dump_sw_setup(playback_handle, out);
	//Prepare the audio for output.
    snd_pcm_prepare (playback_handle);
    
    //Start receiving signal
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
 
#define recv_to_file_args(format) \
    (usrp, format, wirefmt, spb,vgain)
     
    //recv to file
    recv_to_file<std::complex<float> >recv_to_file_args("fc32");
 
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
 
    return EXIT_SUCCESS;
}


template<typename samp_type> void recv_to_file(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &cpu_format,
    const std::string &wire_format,
    size_t samps_per_buff,
    float gain
){
	//Constants
	const int num_audio_samps = samps_per_buff/20;
	const int max_time = samps_per_buff * 1e6 / 640000 + 300;
	//Processing buffers and flags
	char first_run = 1; //Flag for if it is the first buffer to fill or not.
    char current_buf = 0;
    float unwrap_diff;
    float end_val[samps_per_buff - 1], filt_val[samps_per_buff - 1], atan_vals[samps_per_buff];
    float sig_out[num_audio_samps * 2],save_frame[BL]; //Stores Output
    //Counters
    int i, k = 0,j;
	//Metadata, sample buffer
    uhd::rx_metadata_t md;
    samp_type buff[2][samps_per_buff]; //Set up double buffer
	bool overflow_message = true;
	//Keeping time
	int time_block = 0;
    boost::system_time now = boost::get_system_time(), end_loop;	
    boost::posix_time::time_duration block_process;
    //Multithreading
	pthread_t recv_thread;
	proc_recv toRecv;
	
    // initialize save_frame
    for(i = 0; i < BL; i++)
        save_frame[i] = 0;
    //create a receive streamer
    uhd::stream_args_t stream_args(cpu_format,wire_format); //Initialize the format of memory
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args); //Can only be called once.
    toRecv.recv_stream = rx_stream;	
       
    //setup streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = size_t(0);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t(); //holds the time.
    rx_stream->issue_stream_cmd(stream_cmd);   //sends the stream command to initialize.
    
    boost::system_time start = boost::get_system_time(); //Save time that system started
 
    //recieve signals
    while(not stop_signal_called){

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
            if (overflow_message) {
                overflow_message = false;
                std::cerr << boost::format(
                    "Got an overflow indication. Please consider the following:\n"
                    "  Your write medium must sustain a rate of %fMB/s.\n"
                    "  Dropped samples will not be written to the file.\n"
                    "  Please modify this example for your purposes.\n"
                    "  This message will not appear again.\n"
                ) % (usrp->get_rx_rate()*sizeof(samp_type)/1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
			throw std::runtime_error(error);
        }   
        //If time running, recv and do nothing else.
        if (first_run){
        	toRecv.buff = buff[current_buf];
        	toRecv.md = md;
        	toRecv.size = samps_per_buff;
        	pthread_create(&recv_thread, NULL, recvTask, (void *)&toRecv);
        	now = boost::get_system_time();    
        	first_run--;
        }
        //Else, wait for the recv to finish, change buffers, and process last buffer
        else{
        	pthread_join(recv_thread, NULL);
            end_loop = boost::get_system_time(); //.ACTUAL LOCATION FOR END_LOOP
        	current_buf = !current_buf;
        	toRecv.buff = buff[current_buf];
        	pthread_create(&recv_thread, NULL, recvTask, (void *)&toRecv);
        	block_process = end_loop - now;
			if(block_process.ticks() > time_block)
				time_block = block_process.ticks();
			if(block_process.ticks() > max_time)
				std::cout<<"TICKS: "<<block_process.ticks()<<std::endl;
			//Time from start of new receive to start of next recv.
        	now = boost::get_system_time();        
      
            //ATAN2 + UNWRAP: Approx 2000 microsec
            for (i = 0; i < samps_per_buff; i++)
            {
                //ATAN2
                atan_vals[i] = arctan2(buff[!current_buf][i].real(),buff[!current_buf][i].imag());
         
                //UNWRAP (pretty much modulo 2PI conceptually)
                if(i > 0){
                    unwrap_diff = (atan_vals[i]-atan_vals[i-1] + PI) * INV2PI; // Number of times 2 pi goes into the
                }
                else {
                    unwrap_diff = (atan_vals[i]-atan_vals[samps_per_buff - 1] + PI)* INV2PI;
                }
                atan_vals[i] = atan_vals[i] - ((int)unwrap_diff) * 2 * PI;
            }
         
            //LOWPASS FILTER: Approx 4000 microsec
            k = 0;
            for(i = 0; i < (samps_per_buff - 1); i++){
                //COMPUTE DIFF
                end_val[i] = atan_vals[i] - atan_vals[i + 1];
         
                //LOW PASS FILTER (ORD = BL)
                for(j = 0; j < BL; j++){
                    if(j == 0) //If first filter value.
                        filt_val[i] = end_val[i] * B[0];
                    else if( i < j) //If there are less in current frame than the filter order, take some values from the saved frame.
                        filt_val[i] += save_frame[BL + i - j] * B[j];
                    else  //Standard case
                        filt_val[i] += end_val[i - j] * B[j];
         
                }
         
                //save last BL samples.
                if (i >= (samps_per_buff - 1) - BL)
                    save_frame[k++] = end_val[i];
         
            }
         
            //DOWNSAMPLE (modulo is fine on the ARM chip
            k = 0;
            for(i = 0; i < samps_per_buff - 1; i++){
                if(i % 20 == 0){
                    sig_out[k++] = filt_val[i]*gain;
                	sig_out[k++] = filt_val[i]*gain;
                }
            }
        }
        //Output samples
        if(first_run < 1){
        		err = snd_pcm_writei(playback_handle, sig_out, num_audio_samps); //This write drops samples when the audio buffer is full.
//        		Error Reporting : Commented out since ALSA will drop samples
//        		if(err < 0)
//        			std::cerr <<snd_strerror(err)<<std::endl;
        }
		
    } //end while(not stop_signal_called)
    pthread_join(recv_thread, NULL); //Wait for last block to be recorded.
    //post-running wrapping up
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);
 
    std::cout <<"max block execution time (microsec): "<<time_block<<std::endl;
    std::cout << "required execution time (microsec): "<< max_time << std::endl;
    snd_pcm_close (playback_handle);
    std::cout <<"Closed audio"<<std::endl;
 
}

void *recvTask(void* ptr){
	proc_recv  proc = *((proc_recv *) ptr);
	proc.recv_stream->recv(proc.buff, proc.size, proc.md, 3.0, false);
	return ptr;
} 

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time){
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;
 
    boost::system_time start = boost::get_system_time();
    boost::system_time first_lock_time;
 
    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();
 
    while (true) {
        if ((not first_lock_time.is_not_a_date_time()) and
                (boost::get_system_time() > (first_lock_time + boost::posix_time::seconds(setup_time))))
        {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()){
            if (first_lock_time.is_not_a_date_time())
                first_lock_time = boost::get_system_time();
            std::cout << "+";
            std::cout.flush();
        }
        else {
            first_lock_time = boost::system_time(); //reset to 'not a date time'
 
            if (boost::get_system_time() > (start + boost::posix_time::seconds(setup_time))){
                std::cout << std::endl;
                throw std::runtime_error(str(boost::format("timed out waiting for consecutive locks on sensor \"%s\"") % sensor_name));
            }
            std::cout << "_";
            std::cout.flush();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}

//-----------------------------------------------
// Fast arctan2: Jim Shima, 1999/04/23
float arctan2(float y, float x)
{
   float abs_y, angle, r;
   abs_y = fabs(y)+1e-10;      // kludge to prevent 0/0 condition
   if (x>=0)
   {
      r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
   }
   else
   {
      r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
   return(-angle);     // negate if in quad III or IV
   else
   return(angle);
}
