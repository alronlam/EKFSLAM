package util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLog {

	private String filePath;
	private StringBuilder sb;
	private boolean hasWritten;

	public FileLog(String filePath) {
		this.filePath = filePath;
		sb = new StringBuilder();
	}

	public void append(Object o) {
		sb.append(o.toString());
	}

	public void flush() {
		try {
			FileWriter fw = new FileWriter(filePath, hasWritten);
			fw.write(sb.toString());
			sb = new StringBuilder();
			fw.close();

			if (!hasWritten)
				hasWritten = true;

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

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
