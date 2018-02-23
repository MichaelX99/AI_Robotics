Remove-Module R12ROBOT_IK
Import-Module R12ROBOT_IK

# test line
# R12ROBOT_LINE -x1 -1000 -y1 -4000 -x2 -4600 -y2 -104 -height -150 -phi 0 -xspacing 200

Function R12ROBOT_LINE{
Param ([parameter(Mandatory=$true)][int]$x1,
       [parameter(Mandatory=$true)][int]$y1,
       [parameter(Mandatory=$true)][int]$x2,
       [parameter(Mandatory=$true)][int]$y2,
       [parameter(Mandatory=$true)][int]$height,
       [parameter(Mandatory=$true)][int]$phi,
       [parameter(Mandatory=$true)][int]$xspacing)

       $m = ($y2 - $y1)/($x2 - $x1);
       $b = $y1 - ($m * $x1);
        
    For ($i = $x1; $x2 -le $i; $i -= $xspacing){
        $temp = ($m * $i) + $b;
        $CALL = R12ROBOT_IK -x $i -y $temp -z $height -phi $phi
         [pscustomobject]@{
         #m = $m;
         #b = $b;
         newx = $i;
         newy = $temp;
         hand = $CALL.hand;
         wrist = $CALL.wrist;
         elbow = $CALL.elbow;
         shoulder = $CALL.shoulder;
         waist = $CALL.waist;
         COMMAND = $CALL.COMMAND;
         RESPONSE = $CALL.RESPONSE;
         
         }
         
    }

}