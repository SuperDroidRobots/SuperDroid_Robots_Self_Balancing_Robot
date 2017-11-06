
void LCDBegin(){
  lcd.setCursor(0,0);
  lcd.print("Angle:");
  lcd.print(AngleInput, 2);
  
  lcd.setCursor(0,1);
  lcd.print("Speed:");
  lcd.print(SpeedInput, 2);

  lcd.setCursor(0,2);
  lcd.print("Angle to Reach:");
  lcd.print(SpeedOutput, 2);

  lcd.setCursor(0,3);
  lcd.print("Motor Power:");
  lcd.print(AngleOutput, 2);
  
}
void UpdateLCD(){

  lcd.setCursor(6,0);
  if(AngleInput<0)lcd.print(AngleInput, 2);
  else lcd.print(AngleInput, 3);
  lcd.print("  ");
  
  
  lcd.setCursor(6,1);
  if(SpeedInput<0) {lcd.print(SpeedInput);lcd.print("   ");}
  else {lcd.print(SpeedInput);lcd.print("   ");}

  lcd.setCursor(15,2);
  if( SpeedOutput <0)lcd.print(SpeedOutput, 1);
  else lcd.print(SpeedOutput, 1);
  

  lcd.setCursor(12,3);
  if(AngleOutput < 0){lcd.print(AngleOutput, 1);lcd.print(" ");}
  else {lcd.print(AngleOutput, 2);lcd.print("  ");}

  
}

