#ifndef MY_PARAM_H
#define MY_PARAM_H
class Param{
	private:
		std::vector<uint16_t> speed_param{0,0,0,0};
	public:
		Param();
		Param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t accel);
		void set_all_param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t accel);
		void set_first_param(uint16_t speed);
		void set_last_param(uint16_t speed);
		void set_turn_param(uint16_t speed);
		void set_accel_param(uint16_t speed);
		uint16_t get_first_param();
		uint16_t get_last_param();
		uint16_t get_turn_param();
		uint16_t get_accel_param();
};
class ParamList{
	private:
	std::vector<Param> param_list;

	public:
	ParamList() {}
	inline std::vector<Param>::const_iterator begin() const {return param_list.begin(); }
	inline std::vector<Param>::const_iterator end() const { return param_list.end(); }
	inline size_t size() const { return param_list.size(); }
	inline void push_back(const Param& param) { param_list.push_back(param); }
	inline void pop_back() { param_list.pop_back(); }
	const Param &operator[](size_t i) const { return param_list[i]; }
	void setting();
};
#endif
