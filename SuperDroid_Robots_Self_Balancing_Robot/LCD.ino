
void LCDBegin(){
  lcd.setCursor(0,0);
  lcd.print("Angle:");
  lcd.print(AngleInput);
  
  lcd.setCursor(0,1);
  lcd.print("Speed:");
  lcd.print(SpeedInput);

  lcd.setCursor(0,2);
  lcd.print("Angle to Reach:");
  lcd.print(SpeedOutput);

  lcd.setCursor(0,3);
  lcd.print("Motor Power:");
  lcd.print(AngleOutput);
  
}
void UpdateLCD(){

  lcd.setCursor(6,0);
  lcd.print(AngleInput);
  
  lcd.setCursor(6,1);
  lcd.print(SpeedInput);

  lcd.setCursor(15,2);
  lcd.print(SpeedOutput);

  lcd.setCursor(12,3);
  lcd.print(AngleOutput);

  
}

