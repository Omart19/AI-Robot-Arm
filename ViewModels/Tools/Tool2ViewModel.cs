using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Threading;
using Dock.Model.Mvvm.Controls;
using RobotAIArm.Controllers;

namespace RobotAIArm.ViewModels.Tools;

public class Tool2ViewModel : Tool
{
    private MagneticEncoderController _sensorController;

    List<string> angleLabels = new List<string>() {
    "Base Angle:",
    "Lower joint Angle:",
    "Middle joint Angle:",
    "Upper joint Angle:"
    };

    public ObservableCollection<string> EncoderAngles { get; } = new ObservableCollection<string>();
    // Inside Tool2ViewModel

    public Tool2ViewModel()
    {
        if (AppSettings.Instance.IsRemoteMode)
        {
            SignalController.Instance.EncodersReceived += OnEncodersReceived;
        }
        else
        {
            _sensorController = new MagneticEncoderController();
            Task.Run(async () =>
            {
                while (true)
                {
                    await ReadAndUpdateEncoderAngles();
                    await Task.Delay(100);
                }
            });
        }
    }

    private async void OnEncodersReceived(int[] encoders)
    {
        await Dispatcher.UIThread.InvokeAsync(() =>
        {
            EncoderAngles.Clear();
            for (int i = 0; i < encoders.Length; i++)
            {
                EncoderAngles.Add($"{angleLabels[i]} {encoders[i]}");
            }
        });
    }

    private async Task ReadAndUpdateEncoderAngles()
    {
        // Create a list to store the angles
        List<string> anglesList = new List<string>();
        //Console.WriteLine("Reading angles");

        // Read data from each sensor
        anglesList.AddRange(_sensorController.ReadDataFromSensors());

        // Update the ObservableCollection on the UI thread
        await Dispatcher.UIThread.InvokeAsync(() =>
        {
            EncoderAngles.Clear();
            foreach (var (angle, index) in anglesList.Select((value, i) => (value, i)))
            {
                string labeledAngle = $"{angleLabels[index]} {angle}";
                EncoderAngles.Add(labeledAngle);
                //Console.WriteLine($"Added angle: {labeledAngle}"); 
            }
        });
    }
}
