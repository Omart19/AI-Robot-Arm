using System;
using System.ComponentModel;

public class AppSettings : INotifyPropertyChanged
{
    private static AppSettings? _instance;
    public static AppSettings Instance => _instance ??= new AppSettings();

    private bool _isRemoteMode;
    public bool IsRemoteMode
    {
        get => _isRemoteMode;
        set
        {
            if (_isRemoteMode != value)
            {
                _isRemoteMode = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsRemoteMode)));
            }
        }
    }

    public string RemoteIP { get; set; } = "192.168.1.72";

    public event PropertyChangedEventHandler? PropertyChanged;
}
