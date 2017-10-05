#include "mine.h"

void Plot::back_prin(){
//	std::cout<<data.back()[0]<<std::endl;
//	std::cout<<data.back()[1]<<std::endl;
//	std::cout<<data.back()[2]<<std::endl;
//	std::cout<<data.back()[3]<<std::endl;
}
void Plot::prin(){
//	std::cout<<data.front()[0]<<std::endl;
//	std::cout<<data.front()[1]<<std::endl;
//	std::cout<<data.front()[2]<<std::endl;
//	std::cout<<data.front()[3]<<std::endl;
}
template<typename First,typename... Rest>void Plot::push_back(const First& first,const Rest&... rest){
	if(flag == false){
		flag = true;
		data.push_back(std::vector<int>());
	}
	data.back().push_back(first);
	Plot::push_back(rest...);
}
