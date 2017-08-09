int median(int);
int mean(int);
float middle;


/* float alt_read()
{  
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  
  pinMode(pingPin, INPUT);
  
  duration = pulseIn(pingPin, HIGH);
  middle = median(duration / 29 / 2);
  return(middle);

} */

int movingAvg( int senValue)
{
//currValue[filterSize];
int sum = 0;
currValue[i] = senValue;
i++;
for(int j =0; j<filterSize; j++){
sum += currValue[j];}
sum = sum/(filterSize);
if(i>filterSize-1)
{i=0;
}
return(sum);

}

int median( int senValue)
{
//currValue[filterSize];

currValue_median[temp] = senValue;
temp++;

// sort the value of the array
for (int k = 0; k < filterSize - 1; k++){
for (int j = k + 1; j < filterSize-1; j++){
if (currValue_median[k] > currValue_median[j]){
w = currValue_median[k];
currValue_median[k] = currValue_median[j];
currValue_median[j] =w;
}
}
}

// Find the middle value of the sample 
//middle=(fiterSize + 1) / 2;
senValue = currValue_median[(filterSize + 1) / 2];



if(temp>filterSize-1)
{temp=0;
}
return(senValue);


}

