var clamp = func(v, min, max) { v < min ? min : v > max ? max : v }

var warhead_lbs = {
    "aim-120":              44.0,
    "AIM120":               44.0,
    "RB-99":                44.0,
    "aim-7":                88.0,
    "RB-71":                88.0,
    "aim-9":                20.8,
    "RB-24J":               20.8,
    "RB-74":                20.8,
    "R74":                  16.0,
    "MATRA-R530":           55.0,
    "Meteor":               55.0,
    "AIM-54":               135.0,
    "Matra R550 Magic 2":   27.0,
    "Matra MICA":           30.0,
};

var incoming_listener = func {
  var history = getprop("/sim/multiplay/chat-history");
  var hist_vector = split("\n", history);
  if (size(hist_vector) > 0) {
    var last = hist_vector[size(hist_vector)-1];
    var last_vector = split(":", last);
    var author = last_vector[0];
    var callsign = getprop("sim/multiplay/callsign");
    if (size(last_vector) > 1 and author != callsign) {
      # not myself
      #print("not me");
      if (1 == 1) { #  getprop("sim/ja37/armament/damage") == 1) {
        # latest version of failure manager and taking damage enabled
        #print("damage enabled");
        var last1 = split(" ", last_vector[1]);
        if(size(last1) > 2 and last1[size(last1)-1] == "exploded" ) {
          #print("missile hitting someone");
          if (size(last_vector) > 3 and last_vector[3] == " "~callsign) {
            #print("that someone is me!");
            var type = last1[1];
            if (type == "Matra") {
              for (var i = 2; i < size(last1)-1; i += 1) {
                type = type~" "~last1[i];
              }
            }
            var number = split(" ", last_vector[2]);
            var distance = num(number[1]);
            #print(type~"|");
            if(distance != nil) {
              var maxDist = 0;

              if (contains(warhead_lbs, type)) {
                maxDist = maxDamageDistFromWarhead(warhead_lbs[type]);
              } else {
                return;
              }

              var diff = maxDist-distance;
              if (diff < 0) {
                diff = 0;
              }
              
              diff = diff * diff;
              
              var probability = diff / (maxDist*maxDist);

              var failed = fail_systems(probability);
              var percent = 100 * probability;
              print("Took "~percent~"% damage from "~type~" missile at "~distance~" meters distance! "~failed~" systems was hit.");
            }
          } 
        } elsif (last_vector[1] == " KCA cannon shell hit" or last_vector[1] == " Gun Splash On " or last_vector[1] == " M61A1 shell hit") {
          # cannon hitting someone
          #print("cannon");
          if (size(last_vector) > 2 and last_vector[2] == " "~callsign) {
            # that someone is me!
            #print("hitting me");

            var probability = 0.20; # take 20% damage from each hit
            if (last_vector[1] == " Gun Splash On ") {
              probability = 0.30;
            }
            var failed = fail_systems(probability);
            print("Took "~probability*100~"% damage from cannon! "~failed~" systems was hit.");
          }
        }
      }
    }
  }
}

var maxDamageDistFromWarhead = func (lbs) {
  # very simple
  var dist = 3*math.sqrt(lbs);

  return dist;
}

var fail_systems = func (probability) {
    var failure_modes = FailureMgr._failmgr.failure_modes;
    var mode_list = keys(failure_modes);
    var failed = 0;
    foreach(var failure_mode_id; mode_list) {
        if (rand() < probability) {
            FailureMgr.set_failure_level(failure_mode_id, 1);
            failed += 1;
        }
    }
    return failed;
};

setlistener("/sim/multiplay/chat-history", incoming_listener, 0, 0);