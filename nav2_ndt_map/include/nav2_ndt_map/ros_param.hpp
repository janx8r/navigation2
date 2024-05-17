#include <memory>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include <vector>
using namespace std::chrono_literals;

template <class T>
class RosParam
{
private:
    std::string param_name_;
    T value_;
    bool bounds_set = false;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    std::function<void()> param_changed_callback_;
    rcl_interfaces::msg::ParameterDescriptor param_description;

    uint8_t get_rcl_parameter_type()
    {
        if (std::is_same<T, bool>::value)
            return rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        else if (std::is_same<T, int>::value)
            return rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        else if (std::is_same<T, double>::value || std::is_same<T, float>::value)
            return rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        else if (std::is_same<T, std::string>::value)
            return rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        // else if (std::is_same<T, unsigned char *>::value)
        //     return rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
        // else if (std::is_same<T, std::vector<bool>>::value)
        //     return rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
        // else if (std::is_same<T, std::vector<int>>::value)
        //     return rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
        // else if (std::is_same<T, std::vector<double>>::value || std::is_same<T, float *>::value)
        //     return rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        // else if (std::is_same<T, std::vector<std::string>>::value)
        //     return rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
        else
        {
            std::cout << "unknown parameter type" << std::endl;
            return rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
        }
    }

    void constructor(rclcpp::Node *nh, std::string param_name, T default_value, std::string description, std::function<void()> param_changed_callback)
    {
        param_description.name = param_name;
        param_description.type = get_rcl_parameter_type();
        param_description.description = description;
        param_description.read_only = false;
        // param_description.additional_constraints = "Supported values: [jpeg, png]";

        param_name_ = param_name;
        param_changed_callback_ = param_changed_callback;
        nh->declare_parameter(param_name_, default_value, param_description);
        value_ = nh->get_parameter(param_name_).get_value<T>();
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(nh);
        auto cb = [this](const rclcpp::Parameter &p)
        {
            if (p.get_name() == param_name_)
            {
                value_ = p.get_value<T>();
                //std::cout << "changed \"" << p.get_name() << "\" to " << p.get_value<T>() << std::endl;
                if (param_changed_callback_)
                    param_changed_callback_();
            }
        };
        cb_handle_ = param_subscriber_->add_parameter_callback(param_name_, cb);
    }

public:
    RosParam(rclcpp::Node *nh, std::string param_name, T default_value, T min, T max, T step, std::string description = "", std::function<void()> param_changed_callback = nullptr)
    {
        if (default_value < min)
        {
            default_value = min;
            std::cout << "changed default_value of \"" << param_name << "\" to " << default_value << " to match the min (" << min << ") given" << std::endl;
        }
        if (default_value > max)
        {
            default_value = max;
            std::cout << "changed default_value of \"" << param_name << "\" to " << default_value << " to match the max (" << max << ") given" << std::endl;
        }

        if constexpr (std::is_same<T, int>::value)
        {
            if (default_value % step != 0)
            {
                default_value -= default_value % step;
                std::cout << "changed default_value of \"" << param_name << "\" to " << default_value << " to match the step (" << step << ") given" << std::endl;
            }
            if (step != 1)
            {
                step = 1;
                std::cout << "changed the step size of \"" << param_name << "\" to 1, because only this step size is possible for integer parameters." << std::endl;
            }
            rcl_interfaces::msg::IntegerRange integer_range;
            integer_range.from_value = min;
            integer_range.to_value = max;
            integer_range.step = step;
            param_description.integer_range.push_back(integer_range);
        }
        else if constexpr (std::is_same<T, double>::value || std::is_same<T, float>::value)
        {
            T default_value_new = std::round(default_value / step) * step;
            if (abs(default_value - default_value_new) > 1e-10)
            {
                default_value = default_value_new;
                std::cout << "changed default_value of \"" << param_name << "\" to " << default_value << " to match the step (" << step << ") given" << std::endl;
            }

            rcl_interfaces::msg::FloatingPointRange float_range;
            float_range.from_value = min;
            float_range.to_value = max;
            float_range.step = step;
            param_description.floating_point_range.push_back(float_range);
        }

        constructor(nh, param_name, default_value, description, param_changed_callback);
    }

    RosParam(rclcpp::Node *nh, std::string param_name, T default_value, std::string description = "", std::function<void()> param_changed_callback = nullptr)
    {
        constructor(nh, param_name, default_value, description, param_changed_callback);
    }

    T get_val()
    {
        return value_;
    }

    void set_val(T new_value)
    {
        value_ = new_value;
    }
};