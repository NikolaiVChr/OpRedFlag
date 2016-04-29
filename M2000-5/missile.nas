print("*** LOADING missile.nas ... ***");
################################################################################
#
#                     m2005-5's MISSILE MANAGER
#
################################################################################

var AcModel        = props.globals.getNode("sim/model/m2000-5", 1);
var OurHdg         = props.globals.getNode("orientation/heading-deg");
var OurRoll        = props.globals.getNode("orientation/roll-deg");
var OurPitch       = props.globals.getNode("orientation/pitch-deg");
var MPMessaging    = props.globals.getNode("/controls/armament/mp-messaging", 1);
MPMessaging.setBoolValue(0);

var TRUE = 1;
var FALSE = 0;

var g_fps        = 9.80665 * M2FT;
var slugs_to_lbs = 32.1740485564;

var clamp = func(v, min, max) { v < min ? min : v > max ? max : v }

var MISSILE = {
    new: func(p=0, x=0, y=0, z=0, type="nothing"){
        
        var m = { parents : [MISSILE] };
        # Args: p = Pylon.
        # m.status :
        # -1 = stand-by
        # 0 = searching
        # 1 = locked
        # 2 = fired
        m.status            = 0;
        
        # m.free :
        # 0 = status fired with lock
        # 1 = status fired but having lost lock.
        m.free              = 0;
        #m.prop              = AcModel.getNode("systems/armament/missile/").getChild("msl", 0 , 1);
        #m.PylonIndex        = m.prop.getNode("pylon-index", 1).setValue(p);
        m.NameOfMissile     = getprop("sim/weight["~ p ~"]/selected");
        m.PylonIndex        = p;
        m.ID                = p;
        m.pylon_prop        = props.globals.getNode("sim/").getChild("weight", p);
        m.Tgt               = nil;
        m.TgtValid          = nil;
        m.TgtLon_prop       = nil;
        m.TgtLat_prop       = nil;
        m.TgtAlt_prop       = nil;
        m.TgtHdg_prop       = nil;
        m.TgtSpeed_prop     = nil;
        m.TgtPitch_prop     = nil;
        m.update_track_time = 0;
        m.StartTime         = 0;
        m.seeker_dev_e      = 0; # seeker elevation, deg.
        m.seeker_dev_h      = 0; # seeker horizon, deg.
        m.curr_tgt_e        = 0;
        m.curr_tgt_h        = 0;
        m.init_tgt_e        = 0;
        m.init_tgt_h        = 0;
        m.target_dev_e      = 0; # target elevation, deg.
        m.target_dev_h      = 0; # target horizon, deg.
        m.track_signal_e    = 0; # seeker deviation change to keep constant angle (proportional navigation),
        m.track_signal_h    = 0; # this is directly used as input signal for the steering command.
        m.t_coord           = geo.Coord.new().set_latlon(0, 0, 0);
        m.last_t_coord      = m.t_coord;
        #m.next_t_coord      = m.t_coord;
        m.direct_dist_m     = nil;
        m.diveToken      = 0; #this is for cruise missile. when the token is 1, the dive can start....
        m.total_speed_ft = 1;
        m.vApproch       = 1;
        m.tpsApproch     = 0;

        # cruise missiles
        m.nextGroundElevation = 0; # next Ground Elevation
        m.nextGroundElevationMem = [-10000, -1];
        
        # missile specs:
        m.missile_model     = getprop("controls/armament/missile/address");
        m.missile_NoSmoke   = getprop("controls/armament/missile/addressNoSmoke");
        m.missile_Explosion = getprop("controls/armament/missile/addressExplosion");
        m.missile_fov_diam  = getprop("controls/armament/missile/detection-fov-deg");
        m.missile_fov       = m.missile_fov_diam / 2;
        m.max_detect_rng    = getprop("controls/armament/missile/max-detection-rng-nm");
        m.max_seeker_dev    = getprop("controls/armament/missile/track-max-deg") / 2;
        m.force_lbs_1       = getprop("controls/armament/missile/thrust-lbs-1");
        m.force_lbs_2       = getprop("controls/armament/missile/thrust-lbs-2");
        m.stage_1_duration  = getprop("controls/armament/missile/thrust-1-duration-sec");
        m.stage_2_duration  = getprop("controls/armament/missile/thrust-2-duration-sec");
        m.weight_launch_lbs = getprop("controls/armament/missile/weight-launch-lbs");
        m.weight_whead_lbs  = getprop("controls/armament/missile/weight-warhead-lbs");
        m.Cd_base           = getprop("controls/armament/missile/drag-coeff");
        m.eda               = getprop("controls/armament/missile/drag-area");
        m.max_g             = getprop("controls/armament/missile/max-g");
        m.maxExplosionRange = getprop("controls/armament/missile/maxExplosionRange");
        m.maxSpeed          = getprop("controls/armament/missile/maxspeed");
        m.Life              = getprop("controls/armament/missile/life");
        # for messaging but also for the missile's detection capability (not implemented Yet)
        m.fox               = getprop("controls/armament/missile/fox");
        m.rail              = getprop("controls/armament/missile/rail");
        m.cruisealt         = getprop("controls/armament/missile/cruise_alt");
        m.min_speed_for_guiding = getprop("controls/armament/missile/min-guiding-speed-mach");
        m.angular_speed         = getprop("controls/armament/missile/seeker-angular-speed-dps");
        m.arm_time          = getprop("controls/armament/missile/arming-time-sec");
        m.guidance          = getprop("controls/armament/missile/guidance");

        m.last_coord        = nil;

        m.class = m.fox;
        
        # Find the next index for "models/model" and create property node.
        # Find the next index for "ai/models/missile" and create property node.
        # (M. Franz, see Nasal/tanker.nas)
        var n = props.globals.getNode("models", 1);
        for(var i = 0 ; 1 ; i += 1)
        {
            if(n.getChild("model", i, 0) == nil)
            {
                break;
            }
        }
        m.model = n.getChild("model", i, 1);
        var n = props.globals.getNode("ai/models", 1);
        for(var i = 0 ; 1 ; i += 1)
        {
            if(n.getChild("missile", i, 0) == nil)
            {
                break;
            }
        }
        m.ai = n.getChild("missile", i, 1);
        m.ai.getNode("valid", 1).setBoolValue(1);
        var id_model = m.missile_model;
        m.model.getNode("path", 1).setValue(id_model);
        m.life_time = 0;
        
        # create the AI position and orientation properties.
        m.latN   = m.ai.getNode("position/latitude-deg", 1);
        m.lonN   = m.ai.getNode("position/longitude-deg", 1);
        m.altN   = m.ai.getNode("position/altitude-ft", 1);
        m.hdgN   = m.ai.getNode("orientation/true-heading-deg", 1);
        m.pitchN = m.ai.getNode("orientation/pitch-deg", 1);
        m.rollN  = m.ai.getNode("orientation/roll-deg", 1);
        
        # And other AI nodes
        m.trueAirspeedKt  = m.ai.getNode("velocities/true-airspeed-kt", 1);
        m.verticalSpeedFps = m.ai.getNode("velocities/vertical-speed-fps", 1);
        m.myMissileNameProperty = m.ai.getNode("name", 1);
        m.myMissileNameProperty.setValue(m.NameOfMissile);
        m.myPropertyCallsign = m.ai.getNode("callsign", 1);
        m.myPropertyCallsign.setValue("");
        ######## RADAR STUFF ########
        m.inPropertyRange = m.ai.getNode("radar/in-range", 1);
        m.inPropertyRange.setValue(1);
        m.myPropertyRange = m.ai.getNode("radar/range-nm", 1);
        m.myPropertyRange.setValue(0);
        m.myPropertybearing = m.ai.getNode("radar/bearing-deg", 1);
        m.myPropertybearing.setValue(0);
        m.myPropertyelevation = m.ai.getNode("radar/elevation-deg", 1);
        m.myPropertyelevation.setValue(0);
        
        m.ac      = nil;
        m.coord   = geo.Coord.new().set_latlon(0, 0, 0);
        m.s_down  = nil;
        m.s_east  = nil;
        m.s_north = nil;
        m.alt     = nil;
        m.pitch   = 0;
        m.hdg     = nil;
        m.elapsed_last           = 0;

        # for seeker head and stuff
        m.max_g_current = m.max_g;
        m.last_deviation_e = nil;
        m.last_deviation_h = nil;
        m.last_track_e = 0;
        m.last_track_h = 0;

        #aug pro nav:
        m.dist_last = nil;
        m.dist_direct_last = nil;
        m.last_t_course = nil;
        m.last_t_elev_deg = nil;
        m.last_cruise_or_loft = 0;
        m.old_speed_fps = 0;
        m.last_t_norm_speed = nil;
        m.last_t_elev_norm_speed = nil;
        m.last_dt = 0;

        m.g = 0;

        #rail
        m.drop_time = 0;
        m.rail_dist_m = 2.667;#16S210 AIM-9 Missile Launcher
        m.rail_passed = FALSE;
        m.x = 0;
        m.y = 0;
        m.z = 0;
        m.rail_pos = 0;
        m.rail_speed_into_wind = 0;
        
        #SwSoundOnOff.setValue(1);
        #settimer(func(){ SwSoundVol.setValue(vol_search); m.search() }, 1);
        return MISSILE.active[m.ID] = m;
    },

    # this is the dl function : to delete the object when it's not needed anymore
    del: func(){
        #Check proximity one more time...
        #me.poximity_detection();
        me.model.remove();
        me.ai.remove();
        #if(me.free == 1 and me.Tgt != nil) #it might not miss even if it did went free, so commenting this out
        #{
            # just say Missed if it didn't explode
        #    var phrase = me.NameOfMissile ~ " Report : Missed";
        #    if(MPMessaging.getValue() == 1)
        #    {
        #        setprop("/sim/multiplay/chat", phrase);
        #    }
        #    else
        #    {
        #        setprop("/sim/messages/atc", phrase);
        #    }
        #}
        delete(MISSILE.active, me.ID);
    },

    # This function is the way to reload the 3D model : 1)fired missile with smoke, 2)fired missile without smoke, 3)Exploding missile
    reload_model: func(path){
        # Delete the current model
        me.model.remove();
        
        # Find the new model index
        var n = props.globals.getNode("models", 1);
        for(var i = 0 ; 1 ; i += 1)
        {
            if(n.getChild("model", i, 0) == nil)
            {
                break;
            }
        }
        me.model = n.getChild("model", i, 1);
        
        # Put value in model :
        me.model.getNode("path", 1).setValue(path);
        me.model.getNode("latitude-deg-prop", 1).setValue(me.latN.getPath());
        me.model.getNode("longitude-deg-prop", 1).setValue(me.lonN.getPath());
        me.model.getNode("elevation-ft-prop", 1).setValue(me.altN.getPath());
        me.model.getNode("heading-deg-prop", 1).setValue(me.hdgN.getPath());
        me.model.getNode("pitch-deg-prop", 1).setValue(me.pitchN.getPath());
        me.model.getNode("roll-deg-prop", 1).setValue(me.rollN.getPath());
        me.model.getNode("load", 1).remove();
    },

    # get Coord from body position. x,y,z must be in meters.
    getGPS: func(x, y, z) {
        var ac_roll = getprop("orientation/roll-deg");
        var ac_pitch = getprop("orientation/pitch-deg");
        var ac_hdg   = getprop("orientation/heading-deg");

        me.ac = geo.aircraft_position();

        var in = [0,0,0];
        var trans = [[0,0,0],[0,0,0],[0,0,0]];
        var out = [0,0,0];

        in[0] =  -x * M2FT;
        in[1] =  y * M2FT;
        in[2] =  z * M2FT;
        # Pre-process trig functions:
        var cosRx = math.cos(-ac_roll * D2R);
        var sinRx = math.sin(-ac_roll * D2R);
        var cosRy = math.cos(-ac_pitch * D2R);
        var sinRy = math.sin(-ac_pitch * D2R);
        var cosRz = math.cos(ac_hdg * D2R);
        var sinRz = math.sin(ac_hdg * D2R);
        # Set up the transform matrix:
        trans[0][0] =  cosRy * cosRz;
        trans[0][1] =  -1 * cosRx * sinRz + sinRx * sinRy * cosRz ;
        trans[0][2] =  sinRx * sinRz + cosRx * sinRy * cosRz;
        trans[1][0] =  cosRy * sinRz;
        trans[1][1] =  cosRx * cosRz + sinRx * sinRy * sinRz;
        trans[1][2] =  -1 * sinRx * cosRx + cosRx * sinRy * sinRz;
        trans[2][0] =  -1 * sinRy;
        trans[2][1] =  sinRx * cosRy;
        trans[2][2] =  cosRx * cosRy;
        # Multiply the input and transform matrices:
        out[0] = in[0] * trans[0][0] + in[1] * trans[0][1] + in[2] * trans[0][2];
        out[1] = in[0] * trans[1][0] + in[1] * trans[1][1] + in[2] * trans[1][2];
        out[2] = in[0] * trans[2][0] + in[1] * trans[2][1] + in[2] * trans[2][2];
        # Convert ft to degrees of latitude:
        out[0] = out[0] / (366468.96 - 3717.12 * math.cos(me.ac.lat() * D2R));
        # Convert ft to degrees of longitude:
        out[1] = out[1] / (365228.16 * math.cos(me.ac.lat() * D2R));
        # Set submodel initial position:
        var alat = me.ac.lat() + out[0];
        var alon = me.ac.lon() + out[1];
        var aalt = (me.ac.alt() * M2FT) + out[2];
        
        var c = geo.Coord.new();
        c.set_latlon(alat, alon, aalt * FT2M);

        return c;
    },

    clamp: func(v, min, max) { v < min ? min : v > max ? max : v },

    # this function is to convert for the missile from aircraft coordinate to absolute coordinate
    release: func(){
        me.status = 2;
        #me.animation_flags_props();
        
        # Get the A/C position and orientation values.
        me.ac = geo.aircraft_position();
        var ac_roll  = getprop("orientation/roll-deg");
        var ac_pitch = getprop("orientation/pitch-deg");
        var ac_hdg   = getprop("orientation/heading-deg");
        
        me.x = me.pylon_prop.getNode("offsets/x-m").getValue();
        me.y = me.pylon_prop.getNode("offsets/y-m").getValue();
        me.z = me.pylon_prop.getNode("offsets/z-m").getValue();
        
        var init_coord = me.getGPS(me.x, me.y, me.z);

        # Set submodel initial position:
        var alat = init_coord.lat();
        var alon = init_coord.lon();
        var aalt = init_coord.alt() * M2FT;
        me.latN.setDoubleValue(alat);
        me.lonN.setDoubleValue(alon);
        me.altN.setDoubleValue(aalt);
        me.hdgN.setDoubleValue(ac_hdg);
        if (me.rail == FALSE) {
            # align into wind
            var alpha = getprop("orientation/alpha-deg");
            var beta = getprop("orientation/side-slip-deg");# positive is air from right

            var alpha_diff = alpha * math.cos(ac_roll*D2R) + beta * math.sin(ac_roll*D2R);
            alpha_diff = alpha > 0?alpha_diff:0;# not using alpha if its negative to avoid missile flying through aircraft.
            ac_pitch = ac_pitch - alpha_diff;
            
            var beta_diff = beta * math.cos(ac_roll*D2R) * ((ac_roll > 90 or ac_roll < -90)?-1:1) - alpha * math.sin(ac_roll*D2R);
            #ac_hdg = ac_hdg + beta_diff;

            # drop distance in time
            me.drop_time = math.sqrt(2*7/g_fps);# time to fall 7 ft to clear aircraft
        }

        me.pitchN.setDoubleValue(ac_pitch);
        me.rollN.setDoubleValue(ac_roll);
        
        
        me.coord.set_latlon(alat, alon, aalt * FT2M);
        
        me.model.getNode("latitude-deg-prop", 1).setValue(me.latN.getPath());
        me.model.getNode("longitude-deg-prop", 1).setValue(me.lonN.getPath());
        me.model.getNode("elevation-ft-prop", 1).setValue(me.altN.getPath());
        me.model.getNode("heading-deg-prop", 1).setValue(me.hdgN.getPath());
        me.model.getNode("pitch-deg-prop", 1).setValue(me.pitchN.getPath());
        me.model.getNode("roll-deg-prop", 1).setValue(me.rollN.getPath());
        me.model.getNode("load", 1).remove();
        
        # Get initial velocity vector (aircraft):
        me.s_down = getprop("velocities/speed-down-fps");
        me.s_east = getprop("velocities/speed-east-fps");
        me.s_north = getprop("velocities/speed-north-fps");

        if (me.rail == TRUE) {
            var u = getprop("velocities/uBody-fps");# wind from nose
            me.rail_speed_into_wind = u;
        }

        me.alt = aalt;
        me.pitch = ac_pitch;
        me.hdg = ac_hdg;
        
        #me.smoke_prop.setBoolValue(1);
        #SwSoundVol.setValue(0);
        #settimer(func(){ HudReticleDeg.setValue(0) }, 2);
        #interpolate(HudReticleDev, 0, 2);
        
        me.StartTime = props.globals.getNode("/sim/time/elapsed-sec", 1).getValue();
        
        var phrase =  me.fox ~ " at " ~ me.Tgt.get_Callsign() ~ ". Release " ~ me.NameOfMissile;
        if(MPMessaging.getValue() == 1)
        {
            setprop("/sim/multiplay/chat", defeatSpamFilter(phrase));
        }
        else
        {
            setprop("/sim/messages/atc", phrase);
        }
        me.update();
    },

    drag: func (mach) {
        var Cd = 0;
        if (mach < 0.7) {
            Cd = 0.0125 * mach + me.Cd_base;
        } elsif (mach < 1.2 ) {
            Cd = 0.3742 * math.pow(mach, 2) - 0.252 * mach + 0.0021 + me.Cd_base;
        } else {
            Cd = 0.2965 * math.pow(mach, -1.1506) + me.Cd_base;
        }

         return Cd;
    },

    energyBleed: func (gForce, altitude, dt) {
        # Bleed of energy from pulling Gs.
        # This is very inaccurate, but better than nothing.
        #
        # First we get the speedloss including loss due to normal drag:
        var b300 = me.bleed32800at0g(dt);
        var b325 = me.bleed32800at25g(dt)-b300;
        #
        # We then subtract the normal drag.
        var b000 = me.bleed0at0g(dt);
        var b025 = me.bleed0at25g(dt)-b000;
        b300 = 0;
        b000 = 0;
        #
        # We now find what the speedloss will be at sealevel and 32800 ft.
        var speedLoss32800 = b300 + ((gForce-0)/(25-0))*(b325 - b300);
        var speedLoss0 = b000 + ((gForce-0)/(25-0))*(b025 - b000);
        #
        # We then inter/extra-polate that to the currect density-altitude.
        var speedLoss = speedLoss0 + ((altitude-0)/(32800-0))*(speedLoss32800-speedLoss0);
        #
        # For good measure the result is clamped to below zero.
        return clamp(speedLoss, -100000, 0);
    },

    bleed32800at0g: func () {
        var loss_fps = 0 + ((me.last_dt - 0)/(15 - 0))*(-330 - 0);
        return loss_fps*M2FT;
    },

    bleed32800at25g: func () {
        var loss_fps = 0 + ((me.last_dt - 0)/(3.5 - 0))*(-240 - 0);
        return loss_fps*M2FT;
    },

    bleed0at0g: func () {
        var loss_fps = 0 + ((me.last_dt - 0)/(22 - 0))*(-950 - 0);
        return loss_fps*M2FT;
    },

    bleed0at25g: func () {
        var loss_fps = 0 + ((me.last_dt - 0)/(7 - 0))*(-750 - 0);
        return loss_fps*M2FT;
    },

    update: func(){
        # calculate life time of the missile
        var dt = getprop("sim/time/delta-sec");

        var elapsed = systime();

        if (me.elapsed_last != 0) {
            me.dt = (elapsed - me.elapsed_last)*getprop("sim/speed-up");
            if(me.dt <= 0) {
                # to prevent pow floating point error in line:cdm = 0.2965 * math.pow(me.speed_m, -1.1506) + me.cd;
                # could happen if the OS adjusts the clock backwards
                me.dt = 0.00001;
            }
        }
        me.elapsed_last = elapsed;

        var init_launch = 0;
        if(me.life_time > 0)
        {
            init_launch = 1;
        }
        me.life_time += dt;
        # record coords so we can give the latest nearest position for impact.
        me.last_coord = geo.Coord.new().set_latlon(me.coord.lat(), me.coord.lon(), me.coord.alt());
        
        # calculate speed vector before steering corrections.
        
        # Rocket thrust. If dropped, then ignited after fall time of what is the equivalent of 7ft.
        # If the rocket is 2 stage, then ignite the second stage when 1st has burned out.
        var f_lbs = 0;# pounds force (lbf)
        if (me.life_time > me.drop_time) {
            f_lbs = me.force_lbs_1;
        }
        if (me.life_time > me.stage_1_duration + me.drop_time) {
            f_lbs = me.force_lbs_2;
        }
        if (me.life_time > (me.drop_time + me.stage_1_duration + me.stage_2_duration)) {
            f_lbs = 0;
        }

        # this do work for the moment... need to know how to reload a 3D model...
        if(f_lbs < 1 and me.life_time > 2)# need two seconds here, else dropped missiles will lose smoke permently
        {
            var Dapath = me.missile_NoSmoke;
            if(me.model.getNode("path", 1).getValue() != Dapath)
            {
                #print(Dapath);
                me.reload_model(Dapath);
            }
            #print( me.model.getNode("path", 1).getValue());
            #me.smoke_prop.setBoolValue(0);
        }
        
        # kill the AI after a while.
        if(me.life_time > me.Life)
        {
            print("Missile selfdestructed.");
            me.free = 1;
            me.sendMessage(me.NameOfMissile~" missed "~me.Tgt.get_Callsign()~", reason: Selfdestructed.");
            me.animate_explosion();
            me.Tgt = nil;
            settimer(func(){ me.del(); }, 4);
            return;
        }
        # get total speed.
        var d_east_ft  = me.s_east * dt;
        var d_north_ft = me.s_north * dt;
        var d_down_ft  = me.s_down * dt;
        var pitch_deg  = me.pitch;
        var hdg_deg    = me.hdg;
        var dist_h_ft  = math.sqrt((d_east_ft * d_east_ft) + (d_north_ft * d_north_ft));
        var total_s_ft = math.sqrt((dist_h_ft * dist_h_ft) + (d_down_ft * d_down_ft));
        
        if (me.rail == TRUE and me.rail_passed == FALSE) {
            var u = getprop("velocities/uBody-fps");# wind from nose
            var v = getprop("velocities/vBody-fps");# wind from side
            var w = getprop("velocities/wBody-fps");# wind from below

            pitch_deg = getprop("orientation/pitch-deg");
            hdg_deg = getprop("orientation/heading-deg");

            var speed_on_rail = clamp(me.rail_speed_into_wind - u, 0, 1000000);
            var movement_on_rail = speed_on_rail * dt;
            
            me.rail_pos = me.rail_pos + movement_on_rail;
            me.x = me.x - (movement_on_rail * FT2M);# negative cause positive is rear in body coordinates
            #print("rail pos "~(me.rail_pos*FT2M));
        }

        # get air density and speed of sound (fps):
        var rs = environment.rho_sndspeed(me.altN.getValue());
        var rho = rs[0];
        var sound_fps = rs[1];
        
        me.max_g_current = me.max_g+((rho-0.0023769)/(0.00036159-0.0023769))*(me.max_g*0.5909-me.max_g);

        var old_speed_fps = total_s_ft / dt;
        #print("aim "~old_speed_fps);
        #print("ac  "~(getprop("velocities/groundspeed-3D-kt")*KT2FPS));
        me.old_speed_horz_fps = dist_h_ft / dt;
        me.old_speed_fps = old_speed_fps;

        if (me.rail == TRUE and me.rail_passed == FALSE) {
            # if missile is still on rail, we replace the speed, with the speed into the wind from nose on the rail.
            old_speed_fps = me.rail_speed_into_wind;
        }

        me.speed_m = old_speed_fps / sound_fps;
        #print(me.speed_m);
        var Cd = me.drag(me.speed_m);
        
        # Add drag to the total speed using Standard Atmosphere (15C sealevel temperature);
        # rho is adjusted for altitude in environment.rho_sndspeed(altitude),
        # Acceleration = thrust/mass - drag/mass;
        var mass = me.weight_launch_lbs / slugs_to_lbs;
        
        var acc = f_lbs / mass;

        var q = 0.5 * rho * old_speed_fps * old_speed_fps;# dynamic pressure
        var drag_acc = (Cd * q * me.eda) / mass;
        
        # here is a limit for not "Go over" the theoric max speed of the missile
        var speed_fps = 0;
        if(me.speed_m > me.maxSpeed)
        {
             speed_fps = old_speed_fps - drag_acc*dt;
        }
        else
        {
            #Correction made by Necolatis
            speed_fps = old_speed_fps - drag_acc*dt + acc*dt;
        }
        #print("acc: ", acc, " _drag_acc: ", drag_acc);

        if (me.last_dt != 0) {
            speed_fps = speed_fps + me.energyBleed(me.g, me.altN.getValue());
        }
        
        # this is for ground detection fr A/G cruise missile
        if(me.cruisealt != 0 and me.cruisealt < 10000)
        {
            # detect terrain for use in terrain following
            me.nextGroundElevationMem[1] -= 1;
            var geoPlus2 = nextGeoloc(me.coord.lat(), me.coord.lon(), me.hdg, old_speed_fps, dt*5);
            var geoPlus3 = nextGeoloc(me.coord.lat(), me.coord.lon(), me.hdg, old_speed_fps, dt*10);
            var geoPlus4 = nextGeoloc(me.coord.lat(), me.coord.lon(), me.hdg, old_speed_fps, dt*20);
            #var geoPlus5 = nextGeoloc(me.coord.lat(), me.coord.lon(), me.hdg, old_speed_fps, dt*30);
            var e1 = geo.elevation(me.coord.lat(), me.coord.lon());# This is done, to make sure is does not decline before it has passed obstacle.
            var e2 = geo.elevation(geoPlus2.lat(), geoPlus2.lon());# This is the main one.
            var e3 = geo.elevation(geoPlus3.lat(), geoPlus3.lon());# This is an extra, just in case there is an high cliff it needs longer time to climb.
            var e4 = geo.elevation(geoPlus4.lat(), geoPlus4.lon());
            #var e5 = geo.elevation(geoPlus5.lat(), geoPlus5.lon());
            if (e1 != nil) {
                me.nextGroundElevation = e1;
            } else {
                print("nil terrain, blame terrasync! Cruise-missile keeping altitude.");
            }
            if (e2 != nil and e2 > me.nextGroundElevation) {
                me.nextGroundElevation = e2;
                if (e2 > me.nextGroundElevationMem[0] or me.nextGroundElevationMem[1] < 0) {
                    me.nextGroundElevationMem[0] = e2;
                    me.nextGroundElevationMem[1] = 5;
                }
            }
            if (me.nextGroundElevationMem[0] > me.nextGroundElevation) {
                me.nextGroundElevation = me.nextGroundElevationMem[0];
            }
            if (e3 != nil and e3 > me.nextGroundElevation) {
                me.nextGroundElevation = e3;
            }
            if (e4 != nil and e4 > me.nextGroundElevation) {
                me.nextGroundElevation = e4;
            }
            #if (e5 != nil and e5 > me.nextGroundElevation) {
            #   me.nextGroundElevation = e5;
            #}
        }
        
        var grav_bomb = FALSE;
        if (me.force_lbs_1 == 0 and me.force_lbs_2 == 0) {
            grav_bomb = TRUE;
        }

        # Get target position.
        me.t_coord.set_latlon(me.Tgt.get_Latitude(), me.Tgt.get_Longitude(), me.Tgt.get_altitude() * FT2M);

        # guidance
        if(me.status == 2 and me.free == 0 and me.life_time > me.drop_time and grav_bomb == FALSE)
        {
            if(me.rail == FALSE or me.rail_passed == TRUE)
            {
                me.update_track(dt);
            }
            if(me.speed_m < me.min_speed_for_guiding) {
                # it doesn't guide at lower speeds

                me.track_signal_e = 0;
                me.track_signal_h = 0;

                print("Not guiding (too low speed)");
            }
            #print(me.life_time);
            #if(init_launch == 0 )
            #{
                # use the rail or a/c pitch for the first frame.
            #    pitch_deg = getprop("orientation/pitch-deg");
            #}
            #else
            #{
                # here will be set the max angle of pitch and the max angle
                # of heading to avoid G overload
                var myG = steering_speed_G(me.track_signal_e, me.track_signal_h, (total_s_ft / dt), mass, dt);
                if(me.max_g_current < myG)
                {
                    #print("MyG before correction:"~myG);
                    var MyCoef = max_G_Rotation(me.track_signal_e, me.track_signal_h, total_s_ft/dt, mass, dt, me.max_g_current);
                    me.track_signal_e = me.track_signal_e * MyCoef;
                    me.track_signal_h = me.track_signal_h * MyCoef;
                    myG = steering_speed_G(me.track_signal_e, me.track_signal_h, (total_s_ft / dt), mass, dt);
                    #print("MyG after correction:"~myG);
                }
                pitch_deg += me.track_signal_e;
                hdg_deg += me.track_signal_h;
                me.last_track_e = me.track_signal_e;
                me.last_track_h = me.track_signal_h;
                
                #print("Still Tracking : Elevation ", me.track_signal_e, "Heading ", me.track_signal_h, " Gload : ", myG);
            #}
        }
        #print("status :", me.status, "free ", me.free, "init_launch : ", init_launch);
        #print("**Altitude : ", alt_ft, " NextGroundElevation : ", me.nextGroundElevation, "Heading : ", hdg_deg, " **Pitch : ", pitch_deg, "**Speed : ", me.speed_m, " dt :", dt);
        
        # break down total speed to North, East and Down components.
        var speed_down_fps = -math.sin(pitch_deg * D2R) * speed_fps;
        var speed_horizontal_fps = math.cos(pitch_deg * D2R) * speed_fps;
        var speed_north_fps = math.cos(hdg_deg * D2R) * speed_horizontal_fps;
        var speed_east_fps = math.sin(hdg_deg * D2R) * speed_horizontal_fps;
        
        if (grav_bomb == TRUE) {
            # true gravity acc
            speed_down_fps += g_fps * dt;
            pitch_deg = math.atan2( speed_down_fps, speed_horizontal_fps ) * R2D;
        }

        if (me.rail == TRUE and me.rail_passed == FALSE) {
            # missile still on rail, lets calculate its speed relative to the wind coming in from the aircraft nose.
            me.rail_speed_into_wind = me.rail_speed_into_wind + (speed_fps - old_speed_fps);
        }

        # calculate altitude and elevation velocity vector (no incidence here).
        var alt_ft = me.altN.getValue() - ((speed_down_fps + g_fps*dt * !grav_bomb) * dt);
        
        # get horizontal distance and set position and orientation.
        var dist_h_m = speed_horizontal_fps * dt * FT2M;

        if (me.rail == FALSE or me.rail_passed == TRUE) {
            # misssile not on rail, lets move it to next waypoint
            me.coord.apply_course_distance(hdg_deg, dist_h_m);
            me.coord.set_alt(alt_ft * FT2M);
        } else {
            # missile on rail, lets move it on the rail
            speed_fps = me.rail_speed_into_wind;
            me.coord = me.getGPS(me.x, me.y, me.z);
            alt_ft = me.coord.alt() * M2FT;
        }
        me.latN.setDoubleValue(me.coord.lat());
        me.lonN.setDoubleValue(me.coord.lon());
        me.altN.setDoubleValue(alt_ft);
        me.pitchN.setDoubleValue(pitch_deg);
        me.hdgN.setDoubleValue(hdg_deg);
        
        # Velocities Set
        me.trueAirspeedKt.setValue(me.speed_m*661); #Coz the speed is in mach
        me.verticalSpeedFps.setValue(speed_down_fps);
        
        
        
        # Proximity detection
        
        if(me.status == 2 and (me.rail == FALSE or me.rail_passed == TRUE))
        {
            var v = me.poximity_detection();
            if(! v)
            {
                # we exploded, but need a few more secs to spawn
                # the explosion animation.
                settimer(func{me.del();}, 4);
                print("booom");
                return;
            }
            if(me.life_time > 0.5)
            {
                # if not exploded, check if the missile can keep the lock
                if(me.free == 0)
                {
                    me.g = steering_speed_G(me.track_signal_e, me.track_signal_h, (total_s_ft / dt), mass, dt);
                    if(me.g > me.max_g_current)
                    {
                        # target unreachable, fly free.
                        me.free = 1;
                        print("Too much G");
                        # Disable for the moment
                    }
                } else {
                    me.g = 0;
                }
            } else {
                me.g = 0;
            }
            
            # ground interaction²
            var ground = geo.elevation(me.coord.lat(), me.coord.lon());
            #print("Ground :", ground);
            if(ground != nil)
            {
                if(ground > alt_ft*FT2M)
                {
                    print("Missile hit ground");
                    me.free = 1;
                    me.sendMessage(me.NameOfMissile~" missed "~me.Tgt.get_Callsign()~", reason: Hit terrain.");
                    me.animate_explosion();
                    me.Tgt = nil;
                    settimer(func(){ me.del(); }, 4);
                    return;
                }
            }
        } else {
            me.g = 0;
        }

        me.last_t_coord = geo.Coord.new(me.t_coord);

        # record the velocities for the next loop.
        me.s_north = speed_north_fps;
        me.s_east = speed_east_fps;
        me.s_down = speed_down_fps;
        me.total_speed_ft = total_s_ft;
        me.alt = alt_ft;
        me.pitch = pitch_deg;
        me.hdg = hdg_deg;

        if (me.rail_pos > me.rail_dist_m * M2FT) {
            me.rail_passed = TRUE;
            #print("rail passed");
        }

        if(me.life_time < me.Life)
        {
            me.last_dt = dt;
            settimer(func(){ me.update()}, 0);
        }
    },

    update_track: func(dt_ = nil)
    {
        if(me.Tgt == nil)
        {
            return(1);
        }
        if(me.status == 0)
        {
            # Status = searching.
            #me.reset_seeker();
            #SwSoundVol.setValue(vol_search);
            #settimer(func(){ me.search()}, 0.1);
            return(1);
        }
        elsif(me.status == -1)
        {
            # Status = stand-by.
            #me.reset_seeker();
            #SwSoundVol.setValue(0);
            return(1);
        }
        #if(! me.Tgt.Valid.getValue())
        #{
        #    # Lost of lock due to target disapearing:
        #    return to search mode.
        #    me.status = 0;
        #    me.reset_seeker();
        #    SwSoundVol.setValue(vol_search);
        #    settimer(func me.search(), 0.1);
        #    return(1);
        #}
        
        # time interval since lock time or last track loop.
        var time = props.globals.getNode("/sim/time/elapsed-sec", 1).getValue();
        var dt = time - me.update_track_time;
        me.update_track_time = time;
        var last_tgt_e = me.curr_tgt_e;
        var last_tgt_h = me.curr_tgt_h;
        if(me.status == 1)
        {
            # status = locked : get target position relative to our aircraft.
            me.curr_tgt_e = me.Tgt.get_total_elevation(OurPitch.getValue());
            me.curr_tgt_h = me.Tgt.get_deviation(OurHdg.getValue(), geo.aircraft_position());
        } elsif (dt_ != nil) {
            # status = launched : compute target position relative to seeker head.
            
            var t_alt = me.t_coord.alt()*M2FT;
            
            # problem here : We have to calculate de alt difference before
            # calculate the other coord.
            # Prevision of the next position with speed & heading and dt->time to next position
            # Prevision of the next altitude depend on the target appproch on the next second. dt = 0.1
            #me.vApproch;
            
            var next_alt = t_alt - math.sin(me.Tgt.get_Pitch() * D2R) * me.Tgt.get_Speed() * 0.5144 * 0.1;
            
            # nextGeo, depending of the new alt, with a constant speed of the
            # aircraft, 0.2 is the "time"of the precision, in second. This need
            # to be not arbitrary
            
            var nextGeo = nextGeoloc(me.Tgt.get_Latitude(),
                me.Tgt.get_Longitude(),
                me.Tgt.get_heading(),
                me.Tgt.get_Speed() * 0.5144,
                0.1);
            
            #t_alt = next_alt;
            #me.t_coord.set_latlon(nextGeo.lat(), nextGeo.lon(), t_alt * FT2M);
            
            #print("Alt: ", t_alt, " Lat", me.Tgt.get_Latitude(), " Long : ", me.Tgt.get_Longitude());
            
            # Calculate current target elevation and azimut deviation.
            var t_dist_m = me.coord.distance_to(me.t_coord);
            var dist_curr = t_dist_m;
            var dist_curr_direct = me.coord.direct_distance_to(me.t_coord);
            var t_alt_delta_m = (t_alt - me.alt) * FT2M;
            var t_elev_deg = math.atan2(t_alt_delta_m, t_dist_m ) * R2D;
            #var (t_course, dst) = courseAndDistance(me.coord, me.t_coord);
            var t_course = me.coord.course_to(me.t_coord);
            me.curr_tgt_e = t_elev_deg - me.pitch;
            me.curr_tgt_h = t_course - me.hdg;


            var e_gain = 1;
            var h_gain = 1;

            var modulo180 = math.mod(me.curr_tgt_h, 360);
            if(modulo180 > 180)
            {
                modulo180 = -(360 - modulo180);
            }
            if(modulo180 < -180)
            {
                modulo180 = (360 + modulo180);
            }
            me.curr_tgt_h = modulo180;

            # here is how to calculate the own missile detection limitation
            if((math.abs(me.curr_tgt_e) > me.max_seeker_dev)
                or (math.abs(modulo180) > me.max_seeker_dev))
            {
                #print("me.missile_fov:", me.missile_fov, "me.curr_tgt_e:", me.curr_tgt_e, "degree h me.curr_tgt_h:", me.curr_tgt_h, "t_course:", t_course, "me.hdg:", me.hdg, "modulo180:", modulo180);
                e_gain = 0;
                h_gain = 0;
                me.free = 1;
                print("Target is not in missile seeker view anymore");
            }            

            var dev_e = 0;#me.curr_tgt_e;
            var dev_h = 0;#me.curr_tgt_h;

            if (me.last_deviation_e != nil and me.guidance == "heat") {
                # its not our first seeker head move
                # calculate if the seeker can keep up with the angular change of the target

                # missile own movement is subtracted from this change due to seeker being on gyroscope
                
                var dve_dist = me.curr_tgt_e - me.last_deviation_e + me.last_track_e;
                var dvh_dist = me.curr_tgt_h - me.last_deviation_h + me.last_track_h;
                var deviation_per_sec = math.sqrt(dve_dist*dve_dist+dvh_dist*dvh_dist)/dt_;

                if (deviation_per_sec > me.angular_speed) {
                    #print(sprintf("last-elev=%.1f", me.last_deviation_e)~sprintf(" last-elev-adj=%.1f", me.last_track_e));
                    #print(sprintf("last-head=%.1f", me.last_deviation_h)~sprintf(" last-head-adj=%.1f", me.last_track_h));
                    # lost lock due to angular speed limit
                    print(sprintf("%.1f deg/s too big angular change for seeker head.", deviation_per_sec));
                    #print(dt);
                    me.free = 1;
                    e_gain = 0;
                    h_gain = 0;
                }
            }

            me.last_deviation_e = me.curr_tgt_e;
            me.last_deviation_h = me.curr_tgt_h;


            #print("DeltaElevation ", t_alt_delta_m);
            
            ######################################
            ### cruise, loft, cruise-missile   ###
            ######################################

            var loft_angle = 15;# notice Shinobi uses 26.5651 degs, but Raider1 found a source saying 10-20 degs.
            var loft_minimum = 10;# miles
            var cruise_minimum = 10;# miles
            var cruise_or_loft = 0;
            
            if(me.cruisealt != 0 and me.cruisealt < 10000) {
                # this is for Air to ground/sea cruise missile (SCALP, Taurus, Tomahawk...)
                var Daground = 0;# zero for sealevel in case target is ship. Don't shoot A/S missiles over terrain. :)
                if(me.class == "A/G") {
                    Daground = me.nextGroundElevation * M2FT;
                }
                var loft_alt = me.cruisealt;
                if (t_dist_m < me.old_speed_fps * 4 * FT2M and t_dist_m > me.old_speed_fps * 2.5 * FT2M) {
                    # the missile lofts a bit at the end to avoid APN to slam it into ground before target is reached.
                    # end here is between 2.5-4 seconds
                    loft_alt = me.cruisealt*2;
                }
                if (t_dist_m > me.old_speed_fps * 2.5 * FT2M) {# need to give the missile time to do final navigation
                    # it's 1 or 2 seconds for this kinds of missiles...
                    var t_alt_delta_ft = (loft_alt + Daground - me.alt);
                    #print("var t_alt_delta_m : "~t_alt_delta_m);
                    if(loft_alt + Daground > me.alt) {
                        # 200 is for a very short reaction to terrain
                        #print("Moving up");
                        dev_e = -me.pitch + math.atan2(t_alt_delta_ft, me.old_speed_fps * dt_ * 5) * R2D;
                    } else {
                        # that means a dive angle of 22.5° (a bit less 
                        # coz me.alt is in feet) (I let this alt in feet on purpose (more this figure is low, more the future pitch is high)
                        #print("Moving down");
                        var slope = me.clamp(t_alt_delta_ft / 300, -5, 0);# the lower the desired alt is, the steeper the slope.
                        dev_e = -me.pitch + me.clamp(math.atan2(t_alt_delta_ft, me.old_speed_fps * dt_ * 5) * R2D, slope, 0);
                    }
                    cruise_or_loft = 1;
                } elsif (t_dist_m > 500) {
                    # we put 9 feets up the target to avoid ground at the
                    # last minute...
                    #print("less than 1000 m to target");
                    #dev_e = -me.pitch + math.atan2(t_alt_delta_m + 100, t_dist_m) * R2D;
                    #cruise_or_loft = 1;
                } else {
                    #print("less than 500 m to target");
                }
                if (cruise_or_loft == 1) {
                    #print(" pitch "~me.pitch~" + dev_e "~dev_e);
                }
            } elsif (me.cruisealt != 0 and t_dist_m * M2NM > loft_minimum
                 and t_elev_deg < loft_angle #and t_elev_deg > -7.5
                 and me.diveToken == FALSE) {
                # stage 1 lofting: due to target is more than 10 miles out and we havent reached 
                # our desired cruising alt, and the elevation to target is less than lofting angle.
                # The -10 limit, is so the seeker don't lose track of target when lofting.
                if (me.coord.alt() * M2FT < me.cruisealt) {
                    dev_e = -me.pitch + loft_angle;
                    #print(sprintf("Lofting %.1f degs, dev is %.1f", loft_angle, dev_e));
                } else {
                    me.diveToken = TRUE;
                }
                cruise_or_loft = 1;
            } elsif (t_elev_deg < 0 and me.life_time < me.stage_1_duration+me.stage_2_duration+me.drop_time
                     and t_dist_m * M2NM > cruise_minimum) {
                # stage 1/2 cruising: keeping altitude since target is below and more than 5 miles out

                var ratio = (g_fps * dt_)/me.old_speed_fps;
                var attitude = 0;

                if (ratio < 1 and ratio > -1) {
                    attitude = math.asin(ratio)*R2D;
                }

                dev_e = -me.pitch + attitude;
                #print("Cruising");
                cruise_or_loft = 1;
            } elsif (me.last_cruise_or_loft == TRUE and math.abs(me.curr_tgt_e) > 2.5) {
                # after cruising, point the missile in the general direction of the target, before APN starts guiding.
                dev_e = me.curr_tgt_e;
                cruise_or_loft = TRUE;
            }
            
            #print(me.curr_tgt_e);
            
            #var t_course = me.coord.course_to(me.t_coord);
            #me.curr_tgt_h = t_course - me.hdg;
            
            
            

            
            #print("Target Elevation(ft): ", t_alt, " Missile Elevation(ft):", me.alt, " Delta(meters):", t_alt_delta_m);
            # The t_course is false. Prevision is false
            #print("The Target is at: ", t_course, " MyCourse: ", me.hdg, " Delta(degrees): ", me.curr_tgt_h );
            #print("me.curr_tgt_e", me.curr_tgt_e);
            
            # compute gain to reduce target deviation to match an optimum 3 deg
            # this augments steering by an additional 10 deg per second during
            # the trajectory first 2 seconds.
            # then, keep track of deviations at the end of these two initial
            # 2 seconds.
            
            #if(me.rail == "true" or me.life_time > 2)
            #{
            #    if(me.life_time < 3)
            #    {
            #        if(me.curr_tgt_e > 3 or me.curr_tgt_e < -3)
            #        {
            #            e_gain = 1 + (0.1 * dt);
            #        }
            #    }
            #    if(me.curr_tgt_h > 3 or me.curr_tgt_h < -3)
            #    {
            #        h_gain = 1 + (0.1 * dt);
            #    }
            #}
            
            #print(sprintf("curr: elev=%.1f", dev_e)~sprintf(" head=%.1f", dev_h));
            
            ###########################################
            ### augmented proportional navigation   ###
            ###########################################
            if (h_gain != 0 and me.last_dt != 0 and me.dist_last != nil) {
                    var horz_closing_rate_fps = (me.dist_last - dist_curr)*M2FT/me.last_dt;
                    var proportionality_constant = 3;
                    var c_dv = t_course-me.last_t_course;
                    while(c_dv < -180) {
                        c_dv += 360;
                    }
                    while(c_dv > 180) {
                        c_dv -= 360;
                    }
                    var line_of_sight_rate_rps = D2R*c_dv/dt_;

                    #print(sprintf("LOS-rate=%.2f rad/s - closing-rate=%.1f ft/s",line_of_sight_rate_rps,closing_rate_fps));

                    # calculate target acc as normal to LOS line:
                    var t_heading        = me.Tgt.get_heading();
                    var t_pitch          = me.Tgt.get_Pitch();
                    var t_speed          = me.Tgt.get_Speed()*KT2FPS;#true airspeed
                    var t_horz_speed     = math.abs(math.cos(t_pitch*D2R)*t_speed);
                    var t_LOS_norm_head  = t_course + 90;
                    var t_LOS_norm_speed = math.cos((t_LOS_norm_head - t_heading)*D2R)*t_horz_speed;

                    if (me.last_t_norm_speed == nil) {
                        me.last_t_norm_speed = t_LOS_norm_speed;
                    }

                    var t_LOS_norm_acc   = (t_LOS_norm_speed - me.last_t_norm_speed)/dt_;

                    me.last_t_norm_speed = t_LOS_norm_speed;

                    # acceleration perpendicular to instantaneous line of sight in feet/sec^2
                    var acc_sideways_ftps2 = proportionality_constant*line_of_sight_rate_rps*horz_closing_rate_fps+proportionality_constant*t_LOS_norm_acc/2;

                    #print(sprintf("commanded-perpendicular-acceleration=%.1f ft/s^2", acc_sideways_ftps2));

                    # now translate that sideways acc to an angle:
                    var velocity_vector_length_fps = me.old_speed_horz_fps;
                    var commanded_sideways_vector_length_fps = acc_sideways_ftps2*dt_;
                    dev_h = math.atan2(commanded_sideways_vector_length_fps, velocity_vector_length_fps)*R2D;
                    
                    #print(sprintf("horz leading by %.1f deg, commanding %.1f deg", me.curr_tgt_h, dev_h));

                    # when cruising or lofting is controling pitch this if statement should be in effect
                    if (cruise_or_loft == 0) {
                        var vert_closing_rate_fps = (me.dist_direct_last - dist_curr_direct)*M2FT/me.last_dt;
                        var line_of_sight_rate_up_rps = D2R*(t_elev_deg-me.last_t_elev_deg)/dt_;#((me.curr_tgt_e-me.last_tgt_e)*D2R)/dt;
                        # calculate target acc as normal to LOS line: (up acc is positive)
                        var t_approach_bearing             = t_course + 180;
                        var t_horz_speed_away_from_missile = -math.cos((t_approach_bearing - t_heading)*D2R)* t_horz_speed;
                        var t_horz_comp_speed              = math.cos((90+t_elev_deg)*D2R)*t_horz_speed_away_from_missile;
                        var t_vert_comp_speed              = math.sin(t_pitch*D2R)*t_speed*math.cos(t_elev_deg*D2R);
                        var t_LOS_elev_norm_speed          = t_horz_comp_speed + t_vert_comp_speed;

                        if (me.last_t_elev_norm_speed == nil) {
                            me.last_t_elev_norm_speed = t_LOS_elev_norm_speed;
                        }

                        var t_LOS_elev_norm_acc            = (t_LOS_elev_norm_speed - me.last_t_elev_norm_speed)/dt_;
                        me.last_t_elev_norm_speed          = t_LOS_elev_norm_speed;

                        var acc_upwards_ftps2 = proportionality_constant*line_of_sight_rate_up_rps*vert_closing_rate_fps+proportionality_constant*t_LOS_elev_norm_acc/2;
                        velocity_vector_length_fps = me.old_speed_fps;
                        var commanded_upwards_vector_length_fps = acc_upwards_ftps2*dt_;
                        dev_e = math.atan2(commanded_upwards_vector_length_fps, velocity_vector_length_fps)*R2D;
                        #print(sprintf("vert leading by %.1f deg", me.curr_tgt_e));
                    }
            }
            me.dist_last = dist_curr;
            me.dist_direct_last = dist_curr_direct;
            me.last_t_course = t_course;
            me.last_t_elev_deg = t_elev_deg;
            me.last_cruise_or_loft = cruise_or_loft;
            #########################
            #########################


            #print((me.update_track_time-me.StartTime-1)/2);
            # compute target deviation variation then seeker move to keep
            # this deviation constant.
            me.track_signal_e = dev_e * e_gain;
            me.track_signal_h = dev_h * h_gain;
            
            #print(" me.track_signal_e = ", me.track_signal_e, " me.track_signal_h = ", me.track_signal_h);
            #print ("**** curr_tgt_e = ", me.curr_tgt_e, " curr_tgt_h = ", me.curr_tgt_h, " me.track_signal_e = ", me.track_signal_e, " me.track_signal_h = ", me.track_signal_h);
        }
        
        # compute HUD reticle position.
        if(me.status == 1)
        {
            var h_rad = (90 - me.curr_tgt_h) * D2R;
            var e_rad = (90 - me.curr_tgt_e) * D2R;
            #var devs = f14_hud.develev_to_devroll(h_rad, e_rad);
            #var combined_dev_deg = devs[0];
            #var combined_dev_length =  devs[1];
            #var clamped = devs[2];
            #if(clamped)
            #{
            #    SW_reticle_Blinker.blink();
            #}
            #else
            #{
            #    SW_reticle_Blinker.cont();
            #}
            #HudReticleDeg.setValue(combined_dev_deg);
            #HudReticleDev.setValue(combined_dev_length);
        }
        if(me.status != 2 and me.status != -1)
        {
            me.check_t_in_fov();
            # we are not launched yet: update_track() loops by itself at 10 Hz.
            #SwSoundVol.setValue(vol_track);
        }
        return(1);
    },

    
    poximity_detection: func()
    {
        var cur_dir_dist_m = me.coord.direct_distance_to(me.t_coord);
        var BC = cur_dir_dist_m;
        var AC = me.direct_dist_m;
        if(me.last_coord != nil)
        {
            var AB = me.last_coord.direct_distance_to(me.coord);
        }
        # 
        #  A_______C'______ B
        #   \      |      /     We have a system  :   x²   = CB² - C'B²
        #    \     |     /                            C'B  = AB  - AC'
        #     \    |x   /                             AC'² = A'C² + x²
        #      \   |   /
        #       \  |  /        Then, if I made no mistake : x² = BC² - ((BC²-AC²+AB²)/(2AB))²
        #        \ | /
        #         \|/
        #          C
        # C is the target. A is the last missile positioin and B tha actual. 
        # For very high speed (more than 1000 m /seconds) we need to know if,
        # between the position A and the position B, the distance x to the 
        # target is enough short to proxiimity detection.
        
        # get current direct distance.
        #print("me.direct_dist_m = ", me.direct_dist_m);
        
        if(me.direct_dist_m != nil)
        {
            var x2 = BC * BC - (((BC * BC - AC * AC + AB * AB) / (2 * AB)) * ((BC * BC - AC * AC + AB * AB) / (2 * AB)));
            if(BC * BC - x2 < AB * AB)
            {
                # this is to check if AC' < AB
                if(x2 > 0)
                {
                    cur_dir_dist_m = math.sqrt(x2);
                }
                #print(" Dist=", y3, "AC =", AC, " AB=", AB, " BC=", BC);
            }
            #print(me.last_coord.alt());
            #print("cur_dir_dist_m = ", cur_dir_dist_m, " me.direct_dist_m = ", me.direct_dist_m);
            
            if(me.tpsApproch == 0)
            {
                me.tpsApproch = props.globals.getNode("/sim/time/elapsed-sec", 1).getValue();
            }
            else
            {
                me.vApproch = (me.direct_dist_m-cur_dir_dist_m) / (props.globals.getNode("/sim/time/elapsed-sec", 1).getValue() - me.tpsApproch);
                me.tpsApproch = props.globals.getNode("/sim/time/elapsed-sec", 1).getValue();
                #print(me.vApproch);
            }
            
            if(cur_dir_dist_m > me.direct_dist_m and me.direct_dist_m < me.maxExplosionRange * 2 and me.life_time > me.arm_time)
            {
                if(me.direct_dist_m < me.maxExplosionRange)
                {
                    # distance to target increase, trigger explosion.
                    # get missile relative position to the target at last frame.
                    var t_bearing_deg = me.last_t_coord.course_to(me.last_coord);
                    var t_delta_alt_m = me.last_coord.alt() - me.last_t_coord.alt();
                    var new_t_alt_m = me.t_coord.alt() + t_delta_alt_m;
                    var t_dist_m  = math.sqrt(math.abs((me.direct_dist_m * me.direct_dist_m)-(t_delta_alt_m * t_delta_alt_m)));
                    # create impact coords from this previous relative position
                    # applied to target current coord.
                    me.t_coord.apply_course_distance(t_bearing_deg, t_dist_m);
                    me.t_coord.set_alt(new_t_alt_m);
                    var wh_mass = me.weight_whead_lbs;# / slugs_to_lbs;
                    print("FOX2: me.direct_dist_m = ", me.direct_dist_m, " time ", getprop("sim/time/elapsed-sec"));
                    impact_report(me.t_coord, wh_mass, "missile",me.vApproch); # pos, alt, mass_slug, (speed_mps)
                    #var phrase = me.Tgt.get_Callsign() ~ " has been hit by " ~ me.NameOfMissile ~ ". Distance of impact " ~ sprintf( "%01.0f", me.direct_dist_m) ~ " meters";
                    var phrase = sprintf( me.NameOfMissile~" exploded: %01.1f", me.direct_dist_m) ~ " meters from: " ~ me.Tgt.get_Callsign();
                    
                    me.sendMessage(phrase);

                    me.animate_explosion();
                    me.Tgt = nil;
                    return(0);
                }
                else
                {
                    if(me.life_time > 3 and me.free == 0)
                    {
                        # you don't have a second chance. Missile missed
                        me.free = 1;
                    }
                }
            }
        }
        
        me.direct_dist_m = cur_dir_dist_m;
        return(1);
    },

    sendMessage: func (str) {
        if (MPMessaging.getValue()  == 1) {
            setprop("/sim/multiplay/chat", defeatSpamFilter(str));
        } else {
            setprop("/sim/messages/atc", str);
        }
    },    
    
    check_t_in_fov: func(){
        # used only when not launched.
        # compute seeker total angular position clamped to seeker max total
        # angular rotation.
        me.seeker_dev_e += me.track_signal_e;
        me.seeker_dev_e = me.clamp_min_max(me.seeker_dev_e, me.max_seeker_dev);
        me.seeker_dev_h += me.track_signal_h;
        me.seeker_dev_h = me.clamp_min_max(me.seeker_dev_h, me.max_seeker_dev);
        # check target signal inside seeker FOV
        var e_d = me.seeker_dev_e - me.aim9_fov;
        var e_u = me.seeker_dev_e + me.aim9_fov;
        var h_l = me.seeker_dev_h - me.aim9_fov;
        var h_r = me.seeker_dev_h + me.aim9_fov;
        if(me.curr_tgt_e < e_d
            or me.curr_tgt_e > e_u
            or me.curr_tgt_h < h_l
            or me.curr_tgt_h > h_r)
        {
            # target out of FOV while still not launched, return to search loop.
            me.status = 0;
            #settimer(func(){ me.search()}, 2);
            me.Tgt = nil;
            SwSoundVol.setValue(vol_search);
            me.reset_seeker();
        }
        return(1);
    },
    
    search: func(c){
        var tgt = c;
        if(me.status != 2)
        {
            var tempCoord = geo.aircraft_position();
        }
        else
        {
            var tempCoord = me.coord;
        }
        var total_elev  = tgt.get_total_elevation(OurPitch.getValue());    # deg.
        var total_horiz = tgt.get_deviation(OurHdg.getValue(), tempCoord); # deg.
        # check if in range and in the (square shaped here) seeker FOV.
        var abs_total_elev = math.abs(total_elev);
        var abs_dev_deg = math.abs(total_horiz);
        
        me.status = 1;
        me.Tgt = tgt;
        
        me.TgtLon_prop       = me.Tgt.get_Longitude; #getprop("/ai/closest/longitude");
        me.TgtLat_prop       = me.Tgt.get_Latitude;  #getprop("/ai/closest/latitude");
        me.TgtAlt_prop       = me.Tgt.get_altitude;  #getprop("/ai/closest/altitude");
        me.TgtHdg_prop       = me.Tgt.get_heading;   #getprop("/ai/closest/heading");
        #print("TUTUTTUTUTU ", me.Tgt.get_Speed());
        if(me.free == 0 and me.life_time > me.Life)
        {
            settimer(func(){me.update_track()}, 2);
        }
    },
    
    reset_steering: func(){
        me.track_signal_e = 0;
        me.track_signal_h = 0;
    },

    reset_seeker: func(){
        me.curr_tgt_e     = 0;
        me.curr_tgt_h     = 0;
        me.seeker_dev_e   = 0;
        me.seeker_dev_h   = 0;
        #settimer(func(){ HudReticleDeg.setValue(0) }, 2);
        interpolate(HudReticleDev, 0, 2);
        me.reset_steering()
    },
    
    clamp_min_max: func(v, mm){
        if(v < -mm)
        {
            v = -mm;
        }
        elsif(v > mm)
        {
            v = mm;
        }
        return(v);
    },
    
# TODO To be corrected...
    animation_flags_props: func(){
        # create animation flags properties.
        var msl_path = "armament/MatraMICA/flags/msl-id-" ~ me.ID;
        me.msl_prop = props.globals.initNode( msl_path, 1, "BOOL");
        var smoke_path = "armament/MatraMICA/flags/smoke-id-" ~ me.ID;
        me.smoke_prop = props.globals.initNode( smoke_path, 0, "BOOL");
        var explode_path = "armament/MatraMICA/flags/explode-id-" ~ me.ID;
        me.explode_prop = props.globals.initNode( explode_path, 0, "BOOL");
        var explode_smoke_path = "armament/MatraMICA/flags/explode-smoke-id-" ~ me.ID;
        me.explode_smoke_prop = props.globals.initNode( explode_smoke_path, 0, "BOOL");
    },
    
    animate_explosion: func(){
        var Dapath = me.missile_Explosion;
        if(me.model.getNode("path", 1).getValue() != Dapath)
        {
            #print(Dapath);
            me.reload_model(Dapath);
        }
        #me.msl_prop.setBoolValue(0);
        #me.smoke_prop.setBoolValue(0);
        #me.explode_prop.setBoolValue(1);
        #settimer( func(){ me.explode_prop.setBoolValue(0)}, 0.5 );
        #settimer( func(){ me.explode_smoke_prop.setBoolValue(1)}, 0.5 );
        #settimer( func(){ me.explode_smoke_prop.setBoolValue(0)}, 3 );
    },

    active: {},
};

# Create impact report.

# altitde-agl-ft DOUBLE
# impact
#        elevation-m DOUBLE
#        heading-deg DOUBLE
#        latitude-deg DOUBLE
#        longitude-deg DOUBLE
#        pitch-deg DOUBLE
#        roll-deg DOUBLE
#        speed-mps DOUBLE
#        type STRING
# valid "true" BOOL

var impact_report = func(pos, mass_slug, string,speed_mps){
    
    # Find the next index for "ai/models/model-impact" and create property node.
    var n = props.globals.getNode("ai/models", 1);
    for(var i = 0 ; 1 ; i += 1)
    {
        if(n.getChild(string, i, 0) == nil)
        {
            break;
        }
    }
    var impact = n.getChild(string, i, 1);
    
    impact.getNode("impact/elevation-m", 1).setValue(pos.alt());
    impact.getNode("impact/latitude-deg", 1).setValue(pos.lat());
    impact.getNode("impact/longitude-deg", 1).setValue(pos.lon());
    impact.getNode("mass-slug", 1).setValue(mass_slug);
    impact.getNode("speed-mps", 1).setValue(speed_mps);
    impact.getNode("valid", 1).setBoolValue(1);
    impact.getNode("impact/type", 1).setValue("terrain");
    
    var impact_str = "/ai/models/" ~ string ~ "[" ~ i ~ "]";
    setprop("ai/models/model-impact", impact_str);
}

steering_speed_G = func(steering_e_deg, steering_h_deg, s_fps, mass, dt)
{
    # get G number from steering (e, h) in deg, speed in ft/s and mass in slugs.
    var steer_deg = math.sqrt((steering_e_deg * steering_e_deg) + (steering_h_deg * steering_h_deg));
    var radius_ft = math.abs(s_fps / math.cos((90 - steer_deg) * D2R));
    #var g = (mass * s_fps * s_fps / radius_ft * dt) / g_fps;
    var g = (1 / g_fps) *(( s_fps) * (s_fps) / radius_ft);
    #print("#### R = ", radius_ft, " G = ", g);
    return(g);
}

var max_G_Rotation = func(steering_e_deg, steering_h_deg, s_fps, mass, dt, gMax){
    # get G number from steering (e, h) in deg, speed in ft/s and mass in slugs.
    # this function is for calculate the maximum angle without overload G
    
    var steer_deg = math.sqrt((steering_e_deg * steering_e_deg) + (steering_h_deg * steering_h_deg));
    var radius_ft = math.abs(s_fps / math.cos(90 - steer_deg*D2R));
    #var g = (mass * s_fps * s_fps / radius_ft * dt) / g_fps;
    #var g = (1 / g_fps) *( (s_fps/dt) * (s_fps/dt) / radius_ft); s_fps/dt if the function call isn't like that
    var g = (1 / g_fps) *(( s_fps) * (s_fps) / radius_ft);
    
    # isolation of Radius
    #if(s_fps < 1)
    #{
        #s_fps = 1;
    #}
    #var radius_ft2 = (mass * s_fps * s_fps * dt) / ((gMax * 0.9) * g_fps);

    var radius_ft2 =( s_fps * s_fps) / ((gMax * 0.9) * g_fps);
    
    if(math.abs(s_fps/radius_ft2) < 1)
    {
        var steer_rad_theoric = math.acos(math.abs(s_fps / radius_ft2));
        var steer_deg_theoric = 90 - (steer_rad_theoric * R2D);
    }
    else
    {
        var steer_rad_theoric = 1;
        var steer_deg_theoric = 1;
    }
    var radius_ft_th = math.abs(s_fps / math.cos((90 - steer_deg_theoric) * D2R));
    
   # var g_th = (mass * s_fps * s_fps / radius_ft_th * dt) / g_fps;
    var g_th =  ((s_fps * s_fps) / radius_ft_th) / g_fps;
    #print ("Max G ", gMax, " Actual G ", g, "steer_deg_theoric ", steer_deg_theoric);
    return(steer_deg_theoric / steer_deg);
}

# HUD clamped target blinker
# @TODO : use m2000-5 not f-14b
SW_reticle_Blinker = aircraft.light.new("sim/model/f-14b/lighting/hud-sw-reticle-switch", [0.1, 0.1]);
#setprop("sim/model/f-14b/lighting/hud-sw-reticle-switch/enabled", 1);

var nextGeoloc = func(lat, lon, heading, speed, dt, alt=100){
    # lng & lat & heading, in degree, speed in fps
    # this function should send back the futures lng lat
    var distance = speed * dt * FT2M; # should be a distance in meters
    #print("distance ", distance);
    # much simpler than trigo
    var NextGeo = geo.Coord.new().set_latlon(lat, lon, alt);
    NextGeo.apply_course_distance(heading, distance);
    return NextGeo;
}

var MPReport = func(){
    if(MPMessaging.getValue() == 1)
    {
        MPMessaging.setBoolValue(0);
    }
    else
    {
        MPMessaging.setBoolValue(1);
    }
    var phrase = (MPMessaging.getValue()) ? "Activated" : "Desactivated";
    phrase = "MP messaging : " ~ phrase;
    setprop("/sim/messages/atc", phrase);
}

var spams = 0;

var defeatSpamFilter = func (str) {
  spams += 1;
  if (spams == 15) {
    spams = 1;
  }
  str = str~":";
  for (var i = 1; i <= spams; i+=1) {
    str = str~".";
  }
  return str;
}