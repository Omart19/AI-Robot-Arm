using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Threading.Tasks;
using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using Avalonia.Threading;
using RobotAIArm.Controllers;

namespace RobotAIArm.Views.Tools;

public partial class Tool2View : UserControl
{
    
    public Tool2View()
    {
        InitializeComponent();
        

        
    }

    private void InitializeComponent()
    {
        AvaloniaXamlLoader.Load(this);
    }

    
}
