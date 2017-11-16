#ifndef MY_PLOT_H
#define MY_PLOT_H
class Plot{
	private:
		std::vector<std::vector<int>> data;
		bool flag = false;

	public:
		Plot(){}
		void prin();
		void all_print();
		void back_prin();
		inline void clear(){data.clear();}
		inline void push_back(){flag = false;}
		template<typename First,typename... Rest>void push_back(const First& first, const Rest&... rest);
};
template<typename First,typename... Rest>void Plot::push_back(const First& first,const Rest&... rest){
	if(flag == false){
		flag = true;
		data.push_back(std::vector<int>());
	}
	data.back().push_back(first);
	Plot::push_back(rest...);
}
extern Plot plot;
#endif
