package apt.dat;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.CharBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ScrapeAPT {
	
	static Pattern namePattern1 = Pattern.compile(  "(1\\s+\\d+\\s+\\d+\\s+\\d+\\s+\\w+\\s+)(.*)");
	static Pattern namePattern16 = Pattern.compile("(16\\s+\\d+\\s+\\d+\\s+\\d+\\s+\\w+\\s+)(.*)");
	static Pattern namePattern17 = Pattern.compile("(17\\s+\\d+\\s+\\d+\\s+\\d+\\s+\\w+\\s+)(.*)");

	public static void main(String[] args) throws FileNotFoundException, IOException {
		File f = new File("D:/apt.dat");
		List<Base> bases = new ArrayList<>();
		try(BufferedReader br = new BufferedReader(new FileReader(f))) {
			Base current = null;
		    for(String line; (line = br.readLine()) != null; ) {
		    	line = line.strip();
		        if (line.startsWith("1 ")) {//1=airport 16=seaport 17=heliport
		        	if (current != null) {
		        		if (current.isReady()) bases.add(current);
		        		current = null;
		        	}
		        	Matcher matcher = namePattern1.matcher(line);
		        	boolean nameFound = matcher.matches();
		        	if (nameFound) {
		        		current = getCurrent(current);
		        		current.name = matcher.group(2);
		        	}
		        } else if (line.startsWith("16 ")) {//1=airport 16=seaport 17=heliport
		        	if (current != null) {
		        		if (current.isReady()) bases.add(current);
		        		current = null;
		        	}
		        	current = getCurrent(current);
		        	current.sea = true;
		        	Matcher matcher = namePattern16.matcher(line);
		        	boolean nameFound = matcher.matches();
		        	if (nameFound) {
		        		current.name = matcher.group(2);
		        	}
		        } else if (line.startsWith("17 ")) {//1=airport 16=seaport 17=heliport
		        	if (current != null) {
		        		if (current.isReady()) bases.add(current);
		        		current = null;
		        	}
		        	current = getCurrent(current);
		        	current.heli = true;
		        	Matcher matcher = namePattern17.matcher(line);
		        	boolean nameFound = matcher.matches();
		        	if (nameFound) {
		        		current.name = matcher.group(2);
		        	}
		        } else if (line.startsWith("1302 ") && line.contains(" country ")) {//1302=metadata
		        	String[] s = line.split(" country ");
		        	if (s.length == 2) {
		        		current = getCurrent(current);
		        		current.country = s[1].strip();
		        	}		        	
		        } else if (line.startsWith("1302 ") && line.contains(" icao_code ")) {//1302=metadata
		        	String[] s = line.split(" icao_code ");
		        	if (s.length == 2) {
		        		current = getCurrent(current);
		        		current.icao = s[1].strip();
		        	}		        	
		        } else if (line.startsWith("1301 ") && line.toLowerCase().contains("military")) {//1301=ramp metadata
	        		current = getCurrent(current);
	        		current.military = true;
		        } else if (line.startsWith("1300 ") && (line.toLowerCase().contains("fighters") || line.toLowerCase().contains("military"))) {//startup location 
	        		current = getCurrent(current);
	        		current.military = true;
		        } else if (line.startsWith("15 ") && (line.toLowerCase().contains("fighters") || line.toLowerCase().contains("military"))) {//startup location (deprecated)
	        		current = getCurrent(current);
	        		current.military = true;
		        } else if (line.startsWith("20 ") && (line.contains("{@Y}MIL{^r}") || line.contains("{@L}MIL"))) {//signs
	        		current = getCurrent(current);
	        		current.military = true;
		        }
		        
		    }
		    // line is not visible here.
		    if (current != null) {
        		if (current.isReady()) bases.add(current);
        	}
		}
		System.out.println("#\n# License: GPL 2.0 or later, Copyright: Laminar research\n#");
		System.out.println("# Requirements to be in list:\n#  Must have ICAO. Must have country listed. Must have either a MIL sign, parking spots for fighters or runways used for military.");
		System.out.println("# List was auto-generated from APT 1100 file. Heliports are auto commented.\n#");
		System.out.println("var db = {");
		bases.sort(null);
		for (Base base : bases) {
			String code = base.toString();
			String comment = base.toComment();
			int codeSize = code.length();
			if (codeSize < 55) {
				int toAdd = 55 - codeSize;
				code += spaces(toAdd);
			}
			if (base.heli || base.sea) {
				System.out.println("#"+code+comment);
			} else {
				System.out.println(code+comment);
			}
		}
		System.out.println("};");
		System.out.println("# Contains "+bases.size()+" bases");
	}
	
	public static Base getCurrent (Base current) {
		if (current == null) {
			current = new Base();
		}
		return current;
	}
	
	static class Base implements Comparable<Base> {
		String country = null;
		String icao = null;
		boolean military = false;
		String name = null;
		boolean heli = false;
		boolean sea = false;
		public boolean isReady () {
			return icao != null && military;
		}
		@Override
		public String toString() {
			if (country == null) country = "";
			return "\t"+icao+": \""+country+"\",";
		}
		public String toComment() {
			String nm = name==null?"":name;
			String type = heli?"  (heliport)":(sea?"  (seaport)":"");
			return "# "+nm+type;
		}
		@Override
		public int compareTo(Base o) {
			int result = 0;
			if (country != null && o.country != null) {
				result = country.compareToIgnoreCase(o.country);
			}
			if (country != null && o.country == null) {
				result = 1;
			}
			if (country == null && o.country != null) {
				result = -1;
			}
			if (result == 0) {
				return icao.compareToIgnoreCase(o.icao);
			}
			return result;
		}
	}
	
	static String spaces( int spaces ) {
		return CharBuffer.allocate( spaces ).toString().replace( '\0', ' ' );
	}
}
