package util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLog {

	private FileWriter fw;

	public FileLog(String path) {
		try {
			fw = new FileWriter(new File(path));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void append(Object o) {
		try {
			fw.append(o.toString());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void close() {
		try {
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
