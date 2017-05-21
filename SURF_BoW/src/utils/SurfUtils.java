package utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import weka.core.Attribute;
import weka.core.DenseInstance;
import weka.core.Instance;
import weka.core.Instances;

public class SurfUtils {

	public static Instance toInstance(double [] f){
		ArrayList<Attribute> attrInfo = new ArrayList<Attribute>();
		
		for (int i = 0; i < f.length;i++){
			Attribute a = new Attribute(new String("a"+ i));
			attrInfo.add(a);
		}
		
		Instances weka_data = new Instances("data", attrInfo, 0);
		
		Instance inst_i = new DenseInstance(weka_data.numAttributes());
		inst_i.setDataset(weka_data);
		
		for (int j = 0; j < f.length; j++)
			inst_i.setValue(j, f[j]);
		
		return inst_i;
	}
	
	public static Instances toInstances(ArrayList<double[]> data){
		double [] features_example = data.get(0);
		
		ArrayList<Attribute> attrInfo = new ArrayList<Attribute>();
		
		for (int i = 0; i < features_example.length;i++){
			Attribute a = new Attribute(new String("a"+ i));
			attrInfo.add(a);
		}
		
		Instances weka_data = new Instances("data", attrInfo, 0);
		for (int i =0; i < data.size();i++){
			double [] x_i = data.get(i);
			
			Instance inst_i = new DenseInstance(weka_data.numAttributes());
			inst_i.setDataset(weka_data);
			
			for (int j = 0; j < x_i.length; j++)
				inst_i.setValue(j, x_i[j]);
			
			
			weka_data.add(inst_i);
		}
		
		return weka_data;
	}
	
	public static ArrayList<double[]> loadSubsetFeaturesFromFile(String filename, double prob){
		ArrayList<double[]> features = new ArrayList<double[]>();
		
		Random R = new Random();
		
		try {
			BufferedReader BR = new BufferedReader(new FileReader(new File(filename)));
			
			while (true){
				String line = BR.readLine();
				
				if (line == null)
					break;
				
				if (R.nextDouble() < prob){
					String [] tokens = line.split(",");
					
					double [] f_i = new double[tokens.length];
					for (int k = 0; k < tokens.length;k++)
						f_i[k]=Double.parseDouble(tokens[k]);
	
					features.add(f_i);
				}
			}
			
			BR.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return features;	
	}
	
	public static ArrayList<double[]> loadFeaturesFromFile(String filename){
		ArrayList<double[]> features = new ArrayList<double[]>();
		
		try {
			BufferedReader BR = new BufferedReader(new FileReader(new File(filename)));
			
			while (true){
				String line = BR.readLine();
				
				if (line == null)
					break;
				
				String [] tokens = line.split(",");
				
				double [] f_i = new double[tokens.length];
				for (int k = 0; k < tokens.length;k++)
					f_i[k]=Double.parseDouble(tokens[k]);

				features.add(f_i);
				
			}
			
			BR.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return features;	
	}

}
