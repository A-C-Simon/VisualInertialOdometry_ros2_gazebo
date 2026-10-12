#ifdef PACKED
#include "orb_packed_vocabulary.hpp"
#else
#include "ORBVocabulary.h"
#endif
#include <opencv2/opencv.hpp>
#include <openssl/sha.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/resource.h>
struct CheckVocabulary: ORB_SLAM3::ORBVocabulary {
 void hashValid(SHA256_CTX& hash,size_t count) const {
  if(m_nodes.size()<count)throw std::runtime_error("Missing valid nodes");
  for(size_t i=0;i<count;++i){auto &n=m_nodes[i];SHA256_Update(&hash,&n.id,sizeof(n.id));SHA256_Update(&hash,&n.parent,sizeof(n.parent));SHA256_Update(&hash,&n.weight,sizeof(n.weight));if(i)SHA256_Update(&hash,n.descriptor.data,32);for(auto c:n.children)if(c<count)SHA256_Update(&hash,&c,sizeof(c));}
 }
 size_t nodes() const{return m_nodes.size();}
};
double cpu(){timespec t;clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec+t.tv_nsec*1e-9;}
int main(int argc,char**argv){if(argc!=4)return 2;cv::setNumThreads(1);CheckVocabulary v;double start=cpu();if(!v.loadFromTextFile(argv[1]))return 2;double loading=cpu()-start;
 SHA256_CTX hash;SHA256_Init(&hash);v.hashValid(hash,std::stoull(argv[2]));unsigned char digest[32];SHA256_Final(digest,&hash);std::cout<<"valid_node_sha256=";for(auto b:digest)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);std::cout<<std::dec<<"\n";
 cv::RNG rng(922718);cv::Mat descriptor(1,32,CV_8U);SHA256_Init(&hash);for(int i=0;i<30000;++i){rng.fill(descriptor,cv::RNG::UNIFORM,0,256);auto word=v.transform(descriptor);SHA256_Update(&hash,&word,sizeof(word));}
 std::ifstream images(argv[3]);std::string path;size_t queries=30000;auto orb=cv::ORB::create(600);while(std::getline(images,path)){cv::Mat image=cv::imread(path,0),descriptors;std::vector<cv::KeyPoint> keys;orb->detectAndCompute(image,cv::Mat(),keys,descriptors);for(int i=0;i<descriptors.rows;++i){auto word=v.transform(descriptors.row(i));SHA256_Update(&hash,&word,sizeof(word));++queries;}}
 SHA256_Final(digest,&hash);std::cout<<"query_sha256=";for(auto b:digest)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);std::cout<<std::dec<<"\n";rusage r;getrusage(RUSAGE_SELF,&r);std::cout<<"nodes="<<v.nodes()<<" words="<<v.size()<<" queries="<<queries<<" loader_cpu_seconds="<<loading<<" peak_rss_mib="<<r.ru_maxrss/1024.0<<"\n";}
