program Nunchuk;
uses CRT;

procedure ch32I2Cinit( clock: integer);
begin
  port[$F4] := clock mod 256;
  port[$F5] := clock div 256;
end;

procedure ch32I2Cslave (slave: integer);
begin
  if slave <= 127 then
    port[$F6] := slave
  else
    begin
      writeln('Slave: ',slave,' address out of range');
      Halt(1);
    end;
end;

function ch32I2CreadReg (reg: integer): integer;
begin
  port[$F7] := reg;
  ch32I2CreadReg := port[$F8];
end;

procedure ch32I2CwriteReg (reg, value: integer);
begin
  port[$F7] := reg;
  port[$F8] := value;
end;


BEGIN
  ClrScr;

  writeln('Configure I2C clock to 300 KHz');
  ch32I2Cinit(300);

  writeln('Configure I2C slave Nunchuk  ');
  ch32I2Cslave($52);
  ch32I2CwriteReg($F0, $55);

  write('Identify Nunckuk: Nunchuk ');
  if (ch32I2Creadreg($FC) <> $A4) or (ch32I2CreadReg($FD) <> $20) then
    begin
    writeln('not found');
    Halt(2);
    end;
  writeln('found');

  writeln;
  writeln('Press any key to stop');

  gotoXY(1,10); writeln('Move the joystick...');
  while ( not KeyPressed) do
    begin
      gotoXY( 1,11); write('X = ',ch32I2CreadReg(0),'   ');
      gotoXY(15,11); write('Y = ',ch32I2CreadReg(1),'   ');
      gotoXY(30,11); write('Buttons: ');
      if (ch32I2CreadReg(5) and 1) = 0 then
        write('Z')
      else
        write(' ');
      if (ch32I2CreadReg(5) and 2) = 0 then
        write('C')
      else
        write(' ');
    end;
END.
