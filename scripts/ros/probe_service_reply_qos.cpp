#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <iostream>
using namespace eprosima::fastdds::dds;
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  auto* factory=DomainParticipantFactory::get_instance();
  if(factory->load_XML_profiles_file(argv[1])!=ReturnCode_t::RETCODE_OK)return 3;
  auto* participant=factory->create_participant(91,PARTICIPANT_QOS_DEFAULT);
  if(!participant)return 4;
  auto* publisher=participant->create_publisher(PUBLISHER_QOS_DEFAULT);
  if(!publisher){factory->delete_participant(participant);return 5;}
  DataWriterQos defaults,reply;
  auto d=publisher->get_default_datawriter_qos(defaults);
  auto r=publisher->get_datawriter_qos_from_profile("rr/carter2/planner_server/change_stateReply",reply);
  std::cout<<"{\"default_ok\":"<<(d==ReturnCode_t::RETCODE_OK?"true":"false")
    <<",\"default_max_blocking_sec\":"<<defaults.reliability().max_blocking_time.seconds
    <<",\"default_max_blocking_nanosec\":"<<defaults.reliability().max_blocking_time.nanosec
    <<",\"reply_profile_found\":"<<(r==ReturnCode_t::RETCODE_OK?"true":"false")
    <<",\"reply_max_blocking_sec\":"<<reply.reliability().max_blocking_time.seconds
    <<",\"reply_max_blocking_nanosec\":"<<reply.reliability().max_blocking_time.nanosec
    <<",\"reply_reliable\":"<<(reply.reliability().kind==RELIABLE_RELIABILITY_QOS?"true":"false")<<"}"<<std::endl;
  auto deleted=participant->delete_publisher(publisher);
  auto closed=factory->delete_participant(participant);
  return deleted==ReturnCode_t::RETCODE_OK && closed==ReturnCode_t::RETCODE_OK?0:6;
}
