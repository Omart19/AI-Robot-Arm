
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using RobotAIArm.Controllers;
using System;


namespace RobotAIArm.Views.Tools;


public partial class Tool1View : UserControl
{
    private CameraController? _cameraController;
    private ArduinoController _arduinoController = new ArduinoController();
    // private bool _cameraInitialized = false; // Can likely remove this flag now

    public Tool1View()
    {
        InitializeComponent();

        var cameraImage = this.Find<Image>("CameraImage");
        // Create the controller ONCE
        _cameraController = new CameraController(cameraImage, _arduinoController);

        AppSettings.Instance.PropertyChanged += async (s, e) =>
        {
            if (e.PropertyName == nameof(AppSettings.IsRemoteMode))
            {
                bool newMode = AppSettings.Instance.IsRemoteMode;
                Console.WriteLine($"[Tool1View MODE CHANGED Event] New Remote Mode: {newMode}");

                if (_cameraController != null)
                {
                    // Tell the existing controller to change mode
                    Console.WriteLine("[Tool1View MODE CHANGED Event] Calling SetModeAsync...");
                    await _cameraController.SetModeAsync(newMode);
                    Console.WriteLine("[Tool1View MODE CHANGED Event] SetModeAsync call returned.");
                }
                else
                {
                    Console.WriteLine("[Tool1View MODE CHANGED Event] _cameraController is null!");
                }
            }
        };

        // Initial setup based on starting mode (optional but good)
        // Consider calling SetModeAsync here after initialization if needed
        // Loaded event might be better place
    }

    private void InitializeComponent()
    {
        AvaloniaXamlLoader.Load(this);
    }

    // Use OnLoaded or equivalent control lifecycle event for initial setup
    protected override async void OnLoaded(RoutedEventArgs e)
    {
        base.OnLoaded(e);
        if (_cameraController != null)
        {
            Console.WriteLine("[Tool1View OnLoaded] Setting initial camera mode...");
            // Set the initial mode when the view loads
            await _cameraController.SetModeAsync(AppSettings.Instance.IsRemoteMode);
            Console.WriteLine("[Tool1View OnLoaded] Initial camera mode set.");
        }
    }

    // Ensure controller is disposed when the view is detached/unloaded
    protected override void OnUnloaded(RoutedEventArgs e) // Or OnDetachedFromVisualTree
    {
        Console.WriteLine("[Tool1View OnUnloaded] Disposing CameraController...");
        // Use DisposeAsync if possible, otherwise Dispose
        _cameraController?.Dispose(); // Or await _cameraController?.DisposeAsync();
        base.OnUnloaded(e);
    }
}