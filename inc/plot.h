#ifndef MY_PLOT_H
#define MY_PLOT_H
class Plot{
	private:
		std::vector<std::vector<int>> data;
		bool flag = false;

	public:
		Plot(){}
		template<typename First,typename... Rest>void push_back(const First& first, const Rest&... rest);
		inline void push_back(){flag = false;}
		void prin();
		void back_prin();
};
//pomu.push_back(14,5,6,7);
#endif
