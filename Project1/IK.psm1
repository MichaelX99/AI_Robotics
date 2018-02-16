<#
.SYNOPSIS
    A brief description of the function or script. This keyword can be used
    only once in each topic.
.DESCRIPTION
    A detailed description of the function or script. This keyword can be
    used only once in each topic.
.NOTES
    File Name      : xxxx.ps1
    Author         : J.P. Blanc (jean-paul_blanc@silogix-fr.com)
    Prerequisite   : PowerShell V2 over Vista and upper.
    Copyright 2011 - Jean Paul Blanc/Silogix
.LINK
    Script posted over:
    http://silogix.fr
.EXAMPLE
    Example 1
.EXAMPLE
    Example 2
#>

# User Import-Module IK to add this to running powershell session or other files

Function IK{
    Param ([parameter()][int]$x,[parameter()][int]$y,[parameter()][int]$z,[parameter()][int]$phi)
    
    $l = [math]::Sqrt([math]::Pow($x,2)+[math]::Pow($y,2))/10;
    $dx = $l - (113 * [math]::Cos($phi));
    $dy = $z - (113 * [math]::Sin($phi));
    $lambda = [math]::Atan2( ( (-1*$dx) / ([math]::Sqrt( [math]::Pow($dx,2) + [math]::Pow($dy,2) )) ),
                             ( (-1*$dy) / ([math]::Sqrt( [math]::Pow($dx,2) + [math]::Pow($dy,2) )) )
                            );
    $theta_1_pos = $lambda + [math]::Acos((-1*([math]::Pow($dx,2)+[math]::Pow($dy,2)+[math]::Pow(250,2)-[math]::Pow(250,2)))/(2*250*[math]::Sqrt([math]::Pow($dx,2)+[math]::Pow($dy,2))));
    $theta_1_neg = $lambda - [math]::Acos((-1*([math]::Pow($dx,2)+[math]::Pow($dy,2)+[math]::Pow(250,2)-[math]::Pow(250,2)))/(2*250*[math]::Sqrt([math]::Pow($dx,2)+[math]::Pow($dy,2))));
    $theta_2_pos = [math]::Atan2( (($dx-(250*[math]::Cos($theta_1_pos)))/250),
                                  (($dy-(250*[math]::Sin($theta_1_pos)))/250)
                                )-$theta_1_pos;
    $theta_2_neg = [math]::Atan2( (($dx-(250*[math]::Cos($theta_1_neg)))/250),
                                  (($dy-(250*[math]::Sin($theta_1_neg)))/250)
                                )-$theta_1_neg;
    $theta_3_pos = $phi - ($theta_1_pos + $theta_2_pos);
    $theta_3_neg = $phi - ($theta_1_neg + $theta_2_neg);
    $theta_1_pos_deg = $theta_1_pos*180/[math]::PI;
    $theta_1_neg_deg = $theta_1_neg*180/[math]::PI;
    $theta_2_pos_deg = $theta_2_pos*180/[math]::PI;
    $theta_2_neg_deg = $theta_2_neg*180/[math]::PI;
    $theta_3_pos_deg = $theta_3_pos*180/[math]::PI;
    $theta_3_neg_deg = $theta_3_neg*180/[math]::PI;

    $waist = [math]::Truncate([math]::Atan($x/$y)*180/[math]::PI*100);
    $hand = -1*$waist;
    
    $wrist = -1*[math]::Truncate((180+$theta_3_pos_deg)*100);
    $elbow = -1*[math]::Truncate($theta_2_pos_deg*100);
    $shoulder = -1*[math]::Truncate($theta_1_pos_deg*100);
    
    #building the command url
    $urlpasscode= 'PKPMWTlhvllf';
    $urlbase = 'debatedecide.fit.edu/robot.php?o=369&m=Y&p=';
    $urlcmd = '&c=';
    $s = '%20';
    $COMMAND = $urlbase + $urlpasscode + $urlcmd + $hand + $s + $wrist + $s + $elbow + $s + $shoulder + $s + $waist + $s + 'AJMA';
    $RESPONSE = (Invoke-WebRequest $COMMAND).ParsedHtml.body.innerText;

    [pscustomobject]@{
    #x = $x
    #y = $y
    #z = $z
    #phi = $phi
    #l = $l
    #dx = $dx
    #dy = $dy
    #lambda = $lambda
    #theta_1_pos = $theta_1_pos
    #theta_1_neg = $theta_1_neg
    #theta_2_pos = $theta_2_pos
    #theta_2_neg = $theta_2_neg
    #theta_3_pos = $theta_3_pos
    #theta_3_neg = $theta_3_neg
    #theta_1_pos_deg = $theta_1_pos_deg
    #theta_1_neg_deg = $theta_1_neg_deg
    #theta_2_pos_deg = $theta_2_pos_deg
    #theta_2_neg_deg = $theta_2_neg_deg
    #theta_3_pos_deg = $theta_3_pos_deg
    #theta_3_neg_deg = $theta_3_neg_deg
    hand = $hand
    wrist = $wrist
    elbow = $elbow
    shoulder = $shoulder
    waist = $waist
    COMMAND = $COMMAND
    RESPONSE = $RESPONSE
    }
}