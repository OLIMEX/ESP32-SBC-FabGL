program CH32GPIO;

const input = true;
      output = false;
      pullup = true;
      pulldown = false;
      high = true;
      low = false;

function ch32Available: integer;
begin
  ch32Available := port[$F0];
end;

function ch32Version: string;
  var sa,si: string;
begin
  str(port[$FF],sa);
  str(port[$FE],si);
  ch32Version := sa+'.'+si;
end;

procedure ch32GPIOselect( gpio: integer);
begin
  if (gpio > 0) and (gpio < 8) then
    port[$F1] := gpio;
end;

procedure ch32GPIOconfig( inout, pullup: boolean);
begin
  port[$F2] := integer(inout) + integer(pullup) * 2;
end;

procedure ch32GPIOset(level: boolean);
begin
  port[$F3] := integer(level);
end;

function ch32GPIOget: boolean;
begin
  ch32GPIOget := boolean(port[$F3]);
end;

BEGIN
  if (ch32Available = 0) then
    begin
      writeln('CH32 Expander not found')
    end
  else
    begin
      writeln ('CH32 expander firmware version '+ ch32Version + ' found');

      writeln('GPIO 6 = Input');

      ch32GPIOselect(6);
      ch32GPIOconfig(input, pullup);

      writeln('GPIO 7 = Output');

      ch32GPIOselect(7);
      ch32GPIOconfig(output, pullup);

      writeln('GPIO 7 = High');
      ch32GPIOset(high);

      writeln('GPIO 7 = Low');
      ch32GPIOset(low);
    end;
END.
