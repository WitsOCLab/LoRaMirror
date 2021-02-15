// object must have member command:CommandEnum, and optionally steps or resultantmeters and mirrordistance


function Encoder(object, port) {
 var bytes = [];
 var command = 1;
 
 var neg = 0;
 if (object.steps < 0) {
   neg = 1;
   object.steps = -object.steps; //make positive
 }
 
  if (object.command == "tip") {
		bytes[0] = (object.steps & 0xFF00) >> 8;
		bytes[1] = (object.steps & 0x00FF);
		
		return [(1 + neg), bytes[1], bytes[0]]; 
  } 
  else if (object.command == "tilt") {
    bytes[0] = (object.steps & 0xFF00) >> 8;
		bytes[1] = (object.steps & 0x00FF);
		
		return [(3 + neg), bytes[1], bytes[0]]; 
  }
}