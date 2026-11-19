// Deterministic property tests against the actual vendor COBS/CRC codec.
#include "link_protocol.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
struct Received {
 unsigned count=0;link_message_t last{};
 static void callback(const link_message_t*m,void*ctx){auto&r=*static_cast<Received*>(ctx);++r.count;r.last=*m;}
};
int main(){
 std::mt19937 random(20261119);link_decoder_t decoder{};link_decoder_init(&decoder);
 Received received;unsigned cases=20000;
 for(unsigned i=0;i<cases;++i){
  uint16_t size=i%(LINK_MAX_PAYLOAD+1),sequence=random();uint8_t node=1+random()%254,type=random();
  std::vector<uint8_t>payload(size);for(auto&v:payload)v=random();
  uint8_t encoded[LINK_MAX_ENCODED_FRAME];size_t n=link_encode_message(node,type,sequence,payload.data(),size,encoded,sizeof encoded);assert(n);
  unsigned before=received.count;
  for(size_t offset=0;offset<n;){size_t part=std::min(size_t(1+random()%37),n-offset);
   link_decoder_feed(&decoder,encoded+offset,part,Received::callback,&received);offset+=part;}
  assert(received.count==before+1&&received.last.node_id==node&&received.last.type==type&&received.last.sequence==sequence);
  assert(received.last.payload_length==size&&(size==0||memcmp(received.last.payload,payload.data(),size)==0));
  // Corrupt the magic byte inside the first nonzero COBS block; must reject.
  encoded[1]^=1;before=received.count;
  link_decoder_feed(&decoder,encoded,n,Received::callback,&received);assert(received.count==before);
 }
 // Arbitrary garbage/overlong chunks must not corrupt memory; a delimiter and
 // known valid frame must restore reception. Garbage is not sent to the client.
 std::vector<uint8_t>garbage(300000);for(auto&v:garbage)v=random();
 link_decoder_feed(&decoder,garbage.data(),garbage.size(),Received::callback,&received);
 uint8_t zero=0;link_decoder_feed(&decoder,&zero,1,Received::callback,&received);
 uint8_t frame[LINK_MAX_ENCODED_FRAME];size_t n=link_encode_message(1,LINK_CMD_HELLO,0,nullptr,0,frame,sizeof frame);
 unsigned before=received.count;link_decoder_feed(&decoder,frame,n,Received::callback,&received);assert(received.count==before+1);
 printf("PASS protocol stress: %u valid roundtrips, %u corrupt frames rejected, 300000 noise bytes and delimiter recovery\n",cases,cases);
}
