/**
 * @file    neuro_fuzzy_controller.cpp
 * @author  Ashish Ranjan <ranjan.ashish@outlook.com>
 * @version 1.0
 */

#include <floatfann.h>
#include <fann_cpp.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <boost/filesystem.hpp>
using std::cout;
using std::endl;
using std::setw;
using std::left;
using std::right;

#define INIT_EPSILON 0.1

/**
 * Callback function that simply prints the training information to stdout.
 *
 * @param net
 * @param data
 * @param max_epochs
 * @param epochs_between_reports
 * @param desired_error
 * @param epochs
 * @param user_data
 * @return
 */
int printCallback(FANN::neural_net &net, FANN::training_data &data, unsigned int max_epochs, unsigned int epochs_between_reports, float desired_error, unsigned int epochs, void *user_data)
{
    cout << "Epochs      " << setw(8) << epochs << ". " << "Current Error: " << left << net.get_MSE() << right << endl;

    return 0;
}

void neuralNetworkTraining(std::string training_data_file)
{
    /*
     * Parameters for create_standard method.
     *
     * num_layers             : The total number of layers including the input and the output layer.
     * num_input_neurons      : The number of neurons in the input layer.
     * num_hidden_one_neurons : The number of neurons in the first hidden layer.
     * num_hidden_two_neurons : The number of neurons in the second hidden layer.
     * num_output_neurons     : The number of neurons in the output layer.
     */
    const unsigned int num_layers = 4;
    const unsigned int num_input_neurons = 8;
    const unsigned int num_hidden_one_neurons = 15;
    const unsigned int num_hidden_two_neurons = 15;
    const unsigned int num_output_neurons = 1;

    /*
     * Parameters for train_on_data method.
     *
     * max_epochs            : The maximum number of epochs the training should continue.
     * epochs_between_reports: The number of epochs between printing a status report to stdout. A value of zero means no reports should be printed.
     * desired_erros         : The desired get_MSE or get_bit_fail, depending on which stop function is chosen by set_train_stop_function.
     */
    const unsigned int max_epochs = 500000;
    const unsigned int epochs_between_reports = 1000;
    const float desired_error = 0.0003;

    FANN::neural_net net;
    // Create a standard fully connected backpropagation neural network.
    net.create_standard(num_layers, num_input_neurons, num_hidden_one_neurons, num_hidden_two_neurons, num_output_neurons);

    net.set_activation_function_hidden(FANN::SIGMOID_SYMMETRIC_STEPWISE); // Set the activation function for all of the hidden layers.
    net.set_activation_function_output(FANN::SIGMOID_SYMMETRIC_STEPWISE); // Set the activation function for the output layer.
    net.set_training_algorithm(FANN::TRAIN_RPROP);                        // Set the training algorithm.
    net.randomize_weights(-INIT_EPSILON, INIT_EPSILON);                   // Give each connection a random weight between -INIT_EPSILON and INIT_EPSILON.

    cout << endl << "Network Type                         :  ";
    switch (net.get_network_type())
    {
    case FANN::LAYER:
        cout << "LAYER" << endl;
        break;
    case FANN::SHORTCUT:
        cout << "SHORTCUT" << endl;
        break;
    default:
        cout << "UNKNOWN" << endl;
        break;
    }
    net.print_parameters();

    cout << endl << "Training Network." << endl;
    FANN::training_data data;
    if (data.read_train_from_file(training_data_file))
    {
        cout << "Max Epochs: " << setw(8) << max_epochs << ". " << "Desired Error: " << left << desired_error << right << endl;

        net.set_callback(printCallback, NULL);	// Sets the callback function for use during training.
        net.train_on_data(data, max_epochs, epochs_between_reports, desired_error);	// Trains on an entire dataset, for a period of time.

        cout << "Saving Network." << endl;
        net.save("neural_network_controller_float.net");
        unsigned int decimal_point = net.save_to_fixed("neural_network_controller_fixed.net");
        data.save_train_to_fixed("neural_network_controller_fixed.data", decimal_point);
    }
}

int main(int argc, char **argv)
{
    boost::filesystem::path current_dir(boost::filesystem::current_path());
    std::string training_data_file = current_dir.string() + "/training_data.txt";

    if (!boost::filesystem::exists(training_data_file))
    {
        std::cout << "File `" << training_data_file + "` does not exist!\nAborting." << std::endl;
        exit(1);
    }

    neuralNetworkTraining(training_data_file);

    return 0;
}

