var object = {DIR: 0, STEPS: 0, MUL: 1};

function Encoder(object, port) {
  
  //DIR == 1 IS CW1
  if(object.DIR == 1)
  {
    if(object.STEPS >=0 & object.STEPS <=255)
    {
      return[object.DIR, object.STEPS, object.MUL]
    }
  }
  //DIR == 2 IS CCW1
  else if(object.DIR == 2)
  {
    if(object.STEPS >=0 & object.STEPS <=255)
    {
      return[object.DIR, object.STEPS, object.MUL]
    }
  }
  //DIR == 3 IS CW2
  else if(object.DIR == 3)
  {
    if(object.STEPS >=0 & object.STEPS <=255)
    {
      return[object.DIR, object.STEPS, object.MUL]
    }
  }
   //DIR == 4 IS CCW2
  else if(object.DIR == 4)
  {
    if(object.STEPS >=0 & object.STEPS <=255)
    {
      return[object.DIR, object.STEPS, object.MUL]
    }
  }
  
  return [0];
  
}