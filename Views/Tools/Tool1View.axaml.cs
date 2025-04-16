using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using RobotAIArm.Controllers;

namespace RobotAIArm.Views.Tools;

public partial class Tool1View : UserControl
{
    private CameraController? _cameraController;

    public Tool1View()
    {
        InitializeComponent();
        var cameraImage = this.Find<Image>("CameraImage");
        _cameraController = new CameraController(cameraImage);
    }

    private void InitializeComponent()
    {
        AvaloniaXamlLoader.Load(this);
    }

    protected override void OnLoaded(RoutedEventArgs e)
    {
        base.OnLoaded(e);
        _ = _cameraController.StartCameraFeed();
    }

    protected void OnClose()
    {
        _cameraController?.StopCameraFeed(); 
    }
}
