using Avalonia.Controls;
using Avalonia.Input; // Required for KeyEventArgs
using Avalonia.Markup.Xaml;
using RobotAIArm.ViewModels.Tools; // Namespace for your ViewModel

namespace RobotAIArm.Views.Tools
{
    public partial class Tool3View : UserControl
    {
        // Add a field to store the reference if needed outside the constructor
        //private TextBox CommandInputTextBox;

        public Tool3View()
        {
            InitializeComponent();

            // Find the control
            // CommandInputTextBox = this.FindControl<TextBox>("CommandInputTextBox")
            //     ?? throw new Exception("CommandInputTextBox not found in XAML"); // Or handle null more gracefully

            // Attach the event handler here
            CommandInputTextBox.KeyDown += CommandInput_KeyDown;

            // DataContext is set elsewhere in MVVM
        }

        // Keep your existing KeyDown handler method
        private async void CommandInput_KeyDown(object? sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                if (this.DataContext is Tool3ViewModel viewModel && viewModel.SendCommand.CanExecute(null))
                {
                    await viewModel.SendCommand.ExecuteAsync(null);
                }
                e.Handled = true;
            }
        }

        //The InitializeComponent method defined by the partial class generator
        private void InitializeComponent()
        {
            AvaloniaXamlLoader.Load(this);
        }
    }
}