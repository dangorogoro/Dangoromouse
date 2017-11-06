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
void Plot::all_print(){
	std::string text;
	for(auto itr = data.begin();itr != data.end(); ++ itr){
		text.clear();
		for(auto itr2 = (*itr).begin(); itr2 != (*itr).end(); ++itr2){
			text += std::to_string(*itr2) + " ,";
		}
		text.pop_back();
		printf("%s\r\n",text.c_str());
	}
}
Plot plot;
