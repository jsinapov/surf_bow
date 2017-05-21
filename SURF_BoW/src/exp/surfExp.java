package exp;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import utils.SurfUtils;
import weka.clusterers.Cobweb;
import weka.clusterers.SimpleKMeans;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Utils;

public class surfExp {

	public static void buildClustering(int num_trials, int num_objects, String [] behaviors, 
			String surf_data_path, double sample_fraction, int num_visual_words, String kmeans_file){
		
		ArrayList<double[]> features_kmeans = new ArrayList<double[]>();
		
		for (int t = 1; t <= num_trials; t++){
			System.out.println("Loading features from trial "+t);
			for (int o = 1; o <= num_objects; o++){
				for (int b = 0; b < behaviors.length; b++){
					String surf_file_name = new String(surf_data_path+"/"+"obj"+o+"_trial"+t+"_"+behaviors[b]+".surf.txt");
					
					ArrayList<double[]> features_i = SurfUtils.loadSubsetFeaturesFromFile(surf_file_name, sample_fraction);
					//System.out.println("loaded "+features_i.size()+" surf features");
					features_kmeans.addAll(features_i);
				}
			}
			
		}
		System.out.println("Loaded a total of "+features_kmeans.size() +" features for k-means");
		
		//Step 2: apply k-means
		Instances weka_data = SurfUtils.toInstances(features_kmeans);
		
		
		try {
			SimpleKMeans C = new SimpleKMeans();
			C.setNumClusters(num_visual_words);
			
			//Cobweb C = new Cobweb();
			
			C.setDebug(true);
			System.out.println("Building Clustering...");
			C.buildClusterer(weka_data);
			System.out.println("Done. Found "+C.numberOfClusters() + " clusters");
			
			FileOutputStream fout;
			fout = new FileOutputStream(kmeans_file);
			ObjectOutputStream oos = new ObjectOutputStream(fout);
			oos.writeObject(C);
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
	}
	
	public static void computeBoW(int num_trials, int num_objects, String [] behaviors, String kmeans_file, 
			String output_path, String surf_data_path, int num_visual_words, boolean normalize){
		//use clustering to compute BoW representation
		InputStream file;
		try {
					
			//load the clustering
			file = new FileInputStream(kmeans_file);
			InputStream buffer = new BufferedInputStream(file);
			ObjectInput input = new ObjectInputStream (buffer);
			
			
			SimpleKMeans C = (SimpleKMeans)input.readObject();

			//for each interaction, compute and save distribution of visual words
			for (int b = 0; b < behaviors.length; b++){
				System.out.println("Processing behavior "+behaviors[b]);
						
				String norm_tag = "";
				if (normalize)
					norm_tag = new String("_normalized");
				String outfile_b = new String(output_path+"/"+behaviors[b]+"_surf"+num_visual_words+""+norm_tag+".csv");
						
				//open a file to save the results
				FileWriter FW = new FileWriter(outfile_b);
						
				for (int o = 1; o <= num_objects; o++){
					for (int t = 1; t <= num_trials; t++){
								
						String surf_file_name = new String(surf_data_path+"/"+"obj"+o+"_trial"+t+"_"+behaviors[b]+".surf.txt");
								
						//the features for the interaction o b t
						ArrayList<double[]> features = SurfUtils.loadFeaturesFromFile(surf_file_name);
								
						//conts for visual words
						double [] count = new double[num_visual_words];
						for (int i = 0; i < count.length; i++)
							count[i]=0;
								
						//compute the counts
						for (int i = 0; i < features.size(); i++){
							Instance x_i = SurfUtils.toInstance(features.get(i));
							int cluster_x = C.clusterInstance(x_i);
							count[cluster_x]++;
						}
						if (normalize)
							Utils.normalize(count);
								
						FW.write(new String(o+","));
						FW.write(new String(t+","));
						for (int i = 0; i < count.length; i++){
							if (i < count.length-1)
								FW.write(new String(count[i]+","));
							else
								FW.write(new String(count[i]+"\n"));
						}
								
						//save the counts
					}
				}
			FW.close();
						
			}
					
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub

		int num_trials = 5;
		int num_objects = 32;
		
		String [] behaviors = {"look","grasp","lift","hold","lower","drop","push","press"};
		
		double sample_fraction = 0.0005; //2%
		int num_visual_words = 200;
		
		
		String surf_data_path = "/home/jsinapov/research/datasets/ordering_data/surf_data_h400";
		String output_path = "/home/jsinapov/research/datasets/ordering_data/surf_bow";
		String kmeans_file = new String(surf_data_path+"/kmeans_k"+num_visual_words+".obj");
				
		System.out.println("num args = "+args.length);
		System.out.println(args[0]);
		if (args[0].equals("build")){
			System.out.println("Build command");
			surf_data_path = args[1];
			kmeans_file = args[2];
			
			num_visual_words = Integer.parseInt(args[3]);
			sample_fraction = Double.parseDouble(args[4]);

			
			//build clustering -- comment this line out once it's built
			buildClustering(num_trials,num_objects, behaviors, surf_data_path,sample_fraction,num_visual_words, kmeans_file);
			
		}
		else if (args[0].equals("process")){
			System.out.println("Process command");
			surf_data_path = args[1];
			kmeans_file = args[2];
			output_path = args[3];
			num_visual_words = Integer.parseInt(args[4]);
			
			
			System.out.println("Surf data path = "+surf_data_path);
			System.out.println("k-means file: "+kmeans_file);
			System.out.println("output path = "+output_path);
			
			//compute BoW representation  -- comment this line in once the clustering has been built and saved
			computeBoW(num_trials, num_objects, behaviors, kmeans_file, 
					output_path, surf_data_path, num_visual_words, false);
			
		}
		else if (args[0].equals("process_norm")){
			surf_data_path = args[1];
			kmeans_file = args[2];
			output_path = args[3];
			num_visual_words = Integer.parseInt(args[4]);
			
			//compute BoW representation  -- comment this line in once the clustering has been built and saved
			computeBoW(num_trials, num_objects, behaviors, kmeans_file, 
					output_path, surf_data_path, num_visual_words, true);
			
		}
		
		
		
		
		
	   
	}

}
