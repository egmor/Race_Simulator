#include <iostream>
#include <algorithm>
#include <cmath>

namespace constants {
	constexpr double g{ 9.81 }; //ускорение свободного падения
	constexpr double air_density{ 1.225 }; //плотность воздуха в кг/м^3
	constexpr double pi{ 3.14159265358979323846 }; //число Пи
}

namespace temp_const {
	constexpr double weight{ 1400.0 }; //вес автомобиля в кг
	constexpr double grip_tyre_coef{ 1.1 }; //коэффицент сцепления шин
	constexpr double lift_coef{ 1.8 }; //коэффицент прижимной силы
	constexpr double front_area{ 2.2 }; //фронтальная площадь
	constexpr double drag_coef{ 0.7 }; //коэффицент лобового сопротивления

	constexpr double k_front{ 54.2 }; //коэффицент распределения веса на переднюю ось в процентах
	constexpr double k_rear{ 32.7 }; //коэффицент распределения веса на заднюю ось в процентах
}


double force_center(double radius, double velocity) { //радиус дан в метрах
	return temp_const::weight * pow(velocity, 2.0) / radius;
}

double force_grip() {
	return temp_const::grip_tyre_coef * temp_const::weight * constants::g;
}

double basic_limit_velocity(double radius) {
	return pow(temp_const::grip_tyre_coef * constants::g * radius, 0.5);
}

double downforce(double velocity) {
	return 0.5 * constants::air_density * pow(velocity, 2.0) * temp_const::lift_coef * temp_const::front_area;
}

double downforce_limit_velocity(double radius) {
	double numerator{temp_const::grip_tyre_coef * constants::g * radius};
	double denominator_1{ temp_const::grip_tyre_coef * constants::air_density * temp_const::lift_coef * temp_const::front_area * radius / (2 * temp_const::weight)};

	return pow(numerator / (1 - denominator_1), 0.5);
}

double general_aero_coef() {
	double numerator{ constants::air_density * temp_const::front_area * (temp_const::grip_tyre_coef * temp_const::lift_coef + temp_const::drag_coef)};
	double denominator{ 2 * temp_const::weight};

	return numerator / denominator;
}

double brake_dist(double current_velocity, double corner_limit_velocity) {
	double numerator{ (constants::g * temp_const::grip_tyre_coef) + (general_aero_coef() * pow(current_velocity, 2.0)) };
	double denominator{ (constants::g * temp_const::grip_tyre_coef) + (general_aero_coef() * pow(corner_limit_velocity, 2.0)) };

	double brake_distance{ (1.0 / (2 * general_aero_coef())) * std::log(numerator / denominator) };
	if (brake_distance < 0) return 0.0;
	return brake_distance;
}

double central_accel(double velocity, double radius) {
	if (radius <= 0) return 0.0;
	return pow(velocity, 2.0) / radius;
}

double limit_accel(double velocity) {
	return temp_const::grip_tyre_coef * constants::g +
		((temp_const::grip_tyre_coef * constants::air_density * pow(velocity, 2.0) * temp_const::lift_coef * temp_const::front_area) / (2 * temp_const::weight));
}

double full_accel(double velocity, double radius) {
	double limit{ limit_accel(velocity) };
	double central{ central_accel(velocity, radius) };

	if (central < limit) {
		return pow(pow(limit, 2.0) - pow(central, 2.0), 0.5) -
			((limit / temp_const::grip_tyre_coef) - constants::g);
	}
	else return 0.0;
}

double time_straight_to_brake(double lenght, double current_velocity, double corner_limit_velocity, double brake_distance) {
	double dt{ 0.001 };
	double total_time{ 0 };

	if (lenght < brake_dist(current_velocity, corner_limit_velocity)) {
		std::cout << "\nThe car flies out of the turn!\n";
		return -1;
	}

	while (lenght > brake_dist(current_velocity, corner_limit_velocity)) {
		double accel = full_accel(current_velocity, 0.0);

		double temp_velocity{ current_velocity + accel * dt };
		lenght -= ((current_velocity * dt) + (0.5 * accel * pow(dt, 2.0)));
		total_time += dt;
		if (temp_velocity > 350 * 1000.0 / 3600.0) current_velocity = 350 * 1000.0 / 3600.0;
		else current_velocity = temp_velocity;
		if (accel <= 0 && current_velocity < 1.0) {
			break;
		}
	}

	return total_time;
}



int main() {

	double velocity{};
	std::cout << "Enter vehicle speed (km/h): ";
	std::cin >> velocity;
	velocity *= 1000.0 / 3600.0; // переводим в м/с

	double front_brake_balance{};
	std::cout << "Enter front brake balance (0 - 1.0): ";
	std::cin >> front_brake_balance;
	while (front_brake_balance > 1 || front_brake_balance < 0) {
		std::cout << "Invalid input. Please enter a value between 0 and 1: ";
		std::cin >> front_brake_balance;
	}
	double rear_brake_balance{ 1.0 - front_brake_balance };

	std::cout << "\nBrake balance (front|rear): " << front_brake_balance * 100 << "% | " << rear_brake_balance * 100 << "%\n";

	std::cout << "\nEnter force on brake pedal: ";
	double pedal_force{};
	std::cin >> pedal_force;

	std::cout << "\nAerodynamics: " << downforce_limit_velocity(100) << " m/s  |  No aerodynamics: " << basic_limit_velocity(100) << " m/s\n";
	std::cout << "Aerodynamics: " << downforce_limit_velocity(100) * 3.6 << " km/h  |  No aerodynamics: " << basic_limit_velocity(100) * 3.6 << " km/h\n";
	double force_brake_front{ temp_const::k_front * front_brake_balance * pedal_force };
	double force_brake_rear{ temp_const::k_rear * rear_brake_balance * pedal_force };

	double total_brake_force{ force_brake_front + force_brake_rear };

	std::cout << "\nFront brake force: " << force_brake_front << " N\n"
			  << "Rear brake force: " << force_brake_rear << " N\n"
			  << "Total brake force: " << total_brake_force << " N\n";

	std::cout << "\nMaximum possible braking force: " << std::min(temp_const::grip_tyre_coef * (temp_const::weight * constants::g + downforce(velocity)), total_brake_force) << " N\n";

	std::cout << "\nEnter radius next corner: ";
	double radius_corner{};
	std::cin >> radius_corner;

	double corner_velocity{ downforce_limit_velocity(radius_corner) };

	double brake_distance{ brake_dist(velocity, corner_velocity) };

	std::cout << "\nBrake distance in this corner: " << brake_distance << "m\n";

	std::cout << "\nEnter lenght straight before corner: ";
	double lenght_straight{};
	std::cin >> lenght_straight;

	double time_straight{ time_straight_to_brake(lenght_straight, velocity, corner_velocity, brake_distance) };

	if (time_straight > 0) std::cout << "\nTime to the brake point: " << time_straight << "s \n";


	return 0;
}
