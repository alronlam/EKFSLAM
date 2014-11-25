package util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLog {

	private String filePath;
	private StringBuilder sb;

	public FileLog(String filePath) {
		this.filePath = filePath;
		sb = new StringBuilder();
	}

	public void append(Object o) {
		sb.append(o.toString());
	}

	public void writeToFile() {
		try {
			FileWriter fw = new FileWriter(new File(filePath));
			fw.write(sb.toString());
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
