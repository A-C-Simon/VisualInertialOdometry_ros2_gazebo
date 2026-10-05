#pragma once
#include "ORBVocabulary.h"
#include <cmath>
#include <cstdlib>
#include <sstream>
// Specialize the existing loader without changing the vocabulary class layout.
// Keep every valid node, weight and descriptor; row views share one data pool.
// getline prevents the legacy reader from creating a garbage node at EOF.
template<>
inline bool DBoW2::TemplatedVocabulary<cv::Mat,DBoW2::FORB>::loadFromTextFile(const std::string& filename) {
    std::ifstream file(filename);std::string line;
    if(!file || !std::getline(file,line))return false;
    std::istringstream header(line);int scoring,weighting;
    if(!(header>>m_k>>m_L>>scoring>>weighting) || m_k<2 || m_k>20 || m_L<1 || m_L>10 || scoring<0 || scoring>5 || weighting<0 || weighting>3)return false;
    double expected=(std::pow(double(m_k),m_L+1)-1)/(m_k-1);
    if(expected<1 || expected>10000000)return false;
    size_t capacity=static_cast<size_t>(expected);
    m_words.clear();m_nodes.clear();m_scoring=static_cast<DBoW2::ScoringType>(scoring);m_weighting=static_cast<DBoW2::WeightingType>(weighting);createScoringObject();
    m_nodes.reserve(capacity);m_words.reserve(static_cast<size_t>(std::pow(double(m_k),m_L)));m_nodes.resize(1);m_nodes[0].id=0;
    cv::Mat descriptors(static_cast<int>(capacity),DBoW2::FORB::L,CV_8UC1);
    while(std::getline(file,line)) {
        if(line.find_first_not_of(" \t\r")==std::string::npos)continue;
        if(m_nodes.size()>=capacity)return false;
        const char* cursor=line.c_str();
        auto integer=[&](unsigned& value) {
            while(*cursor==' ' || *cursor=='\t' || *cursor=='\r')++cursor;
            if(*cursor<'0' || *cursor>'9')return false;
            unsigned n=0;
            do{if(n>100000000)return false;n=n*10+(*cursor-'0');++cursor;}while(*cursor>='0'&&*cursor<='9');
            value=n;return true;
        };
        unsigned parent,leaf;
        if(!integer(parent)||!integer(leaf)||parent>=m_nodes.size()||leaf>1)return false;
        auto nid=m_nodes.size();m_nodes.emplace_back();auto& node=m_nodes.back();node.id=nid;node.parent=parent;m_nodes[parent].children.push_back(nid);
        node.descriptor=descriptors.row(static_cast<int>(nid));auto* data=node.descriptor.ptr<unsigned char>();
        for(int k=0;k<DBoW2::FORB::L;++k){unsigned v;if(!integer(v)||v>255)return false;data[k]=v;}
        char* end=nullptr;node.weight=std::strtod(cursor,&end);if(end==cursor || !std::isfinite(node.weight))return false;
        while(*end==' ' || *end=='\t' || *end=='\r')++end;if(*end)return false;
        if(leaf){node.word_id=m_words.size();m_words.push_back(&node);}else node.children.reserve(m_k);
    }
    return !m_words.empty();
}
