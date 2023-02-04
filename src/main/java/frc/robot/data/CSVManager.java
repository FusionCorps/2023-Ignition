package frc.robot.data;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class CSVManager {
    File file;
    FileWriter writer;
    String filePath;

    String ogContent = "";
    public CSVManager(String fileName) {
        filePath = fileName;
        file = new File(fileName + ".csv");
        try {
            if (file.createNewFile()) {
                System.out.println("File created :)");
            } else {
                System.out.println("File already exists");
            }
            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                ogContent += line + "\n";
            }
            scanner.close();
            writer = new FileWriter(file);
            writer.write(ogContent);
        } catch (IOException e) {
            System.out.println("Error :(");
            e.printStackTrace();
        }
    }
    public void logDataPoint(double x, double y) {
        try {
            writer.write(x + ", " + y+"\n");
        } catch (IOException e) {
            System.out.println("Error :(");
            e.printStackTrace();
        }
    }

    public void close() {
        try {
            writer.close();
        } catch (IOException e) {
            System.out.println("Error :(");
            e.printStackTrace();
        }
    }

}
