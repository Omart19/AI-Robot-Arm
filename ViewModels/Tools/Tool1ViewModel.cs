using Dock.Model.Mvvm.Controls;
using RobotAIArm.Controllers;

namespace RobotAIArm.ViewModels.Tools;

public class Tool1ViewModel : Tool
{
    public ArduinoController Arduino { get; }

    public Tool1ViewModel(ArduinoController arduino)
    {
        Arduino = arduino;
    }
}
