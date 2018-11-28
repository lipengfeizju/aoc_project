/*
 * ../bin/train_FFLDDetector --name chair --result chair_model.txt --nb-negatives 200 --annotation-dir /media/natanaso/Data/Stuff/Research/git/catkin_ws/src/nx_pkg/ffld_nx/data/PascalFormat/mine/Annotations /media/natanaso/Data/Stuff/Research/git/catkin_ws/src/nx_pkg/ffld_nx/data/PascalFormat/mine/JPEGImages
 */




#include "SimpleOpt.h"

#include "Mixture.h"

#include <libxml/parser.h>
#include <boost/filesystem.hpp>   // includes all needed Boost.Filesystem declarations
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

// SimpleOpt array of valid options
enum
{
	OPT_C, OPT_DATAMINE, OPT_INTERVAL, OPT_HELP, OPT_J, OPT_RELABEL, OPT_MODEL, OPT_NAME,
	OPT_PADDING, OPT_RESULT, OPT_SEED, OPT_OVERLAP, OPT_NB_COMP, OPT_NB_NEG, OPT_AN_DIR
};

CSimpleOpt::SOption SOptions[] =
{
	{ OPT_C, "-c", SO_REQ_SEP },
	{ OPT_C, "--C", SO_REQ_SEP },
	{ OPT_DATAMINE, "-d", SO_REQ_SEP },
	{ OPT_DATAMINE, "--datamine", SO_REQ_SEP },
	{ OPT_INTERVAL, "-e", SO_REQ_SEP },
	{ OPT_INTERVAL, "--interval", SO_REQ_SEP },
	{ OPT_HELP, "-h", SO_NONE },
	{ OPT_HELP, "--help", SO_NONE },
	{ OPT_J, "-j", SO_REQ_SEP },
	{ OPT_J, "--J", SO_REQ_SEP },
	{ OPT_RELABEL, "-l", SO_REQ_SEP },
	{ OPT_RELABEL, "--relabel", SO_REQ_SEP },
	{ OPT_MODEL, "-m", SO_REQ_SEP },
	{ OPT_MODEL, "--model", SO_REQ_SEP },
	{ OPT_NAME, "-n", SO_REQ_SEP },
	{ OPT_NAME, "--name", SO_REQ_SEP },
	{ OPT_PADDING, "-p", SO_REQ_SEP },
	{ OPT_PADDING, "--padding", SO_REQ_SEP },
	{ OPT_RESULT, "-r", SO_REQ_SEP },
	{ OPT_RESULT, "--result", SO_REQ_SEP },
	{ OPT_SEED, "-s", SO_REQ_SEP },
	{ OPT_SEED, "--seed", SO_REQ_SEP },
	{ OPT_OVERLAP, "-v", SO_REQ_SEP },
	{ OPT_OVERLAP, "--overlap", SO_REQ_SEP },
	{ OPT_NB_COMP, "-x", SO_REQ_SEP },
	{ OPT_NB_COMP, "--nb-components", SO_REQ_SEP },
	{ OPT_NB_NEG, "-z", SO_REQ_SEP },
	{ OPT_NB_NEG, "--nb-negatives", SO_REQ_SEP },
	{ OPT_AN_DIR, "-a", SO_REQ_SEP },
	{ OPT_AN_DIR, "--annotation-dir", SO_REQ_SEP },	
	SO_END_OF_OPTIONS
};

void showUsage()
{
	std::cout << "Usage: train [options] image_dir\n\n"
			"Options:\n"
			"  -c,--C <arg>             SVM regularization constant (default 0.002)\n"
			"  -d,--datamine <arg>      Maximum number of data-mining iterations within each "
			"training iteration  (default 10)\n"
			"  -e,--interval <arg>      Number of levels per octave in the HOG pyramid (default 5)"
			"\n"
			"  -h,--help                Display this information\n"
			"  -j,--J <arg>             SVM positive regularization constant boost (default 2)\n"
			"  -l,--relabel <arg>       Maximum number of training iterations (default 8, half if "
			"no part)\n"
			"  -m,--model <file>        Read the initial model from <file> (default zero model)\n"
			"  -n,--name <arg>          Name of the object to detect (default \"person\")\n"
			"  -p,--padding <arg>       Amount of zero padding in HOG cells (default 6)\n"
			"  -r,--result <file>       Write the trained model to <file> (default \"model.txt\")\n"
			"  -s,--seed <arg>          Random seed (default time(NULL))\n"
			"  -v,--overlap <arg>       Minimum overlap in latent positive search (default 0.7)\n"
			"  -x,--nb-components <arg> Number of mixture components (without symmetry, default 3)\n"
			"  -z,--nb-negatives <arg>  Maximum number of negative images to consider (default all)\n"
			"  -a,--annotation-dir <arg>Path to the directory containing the .xml annotations for the "
			"supplied image_dir (default .)"
		 << std::endl;
}

template <typename Result>
static inline Result content(const xmlNodePtr cur)
{
	if ((cur == NULL) || (cur->xmlChildrenNode == NULL))
		return Result();
	
	std::istringstream iss(reinterpret_cast<const char *>(cur->xmlChildrenNode->content));
	Result result;
	iss >> result;
	return result;
}

int parseAnnotationXML( const std::string & filename,
                        int &width_, int &height_, int &depth_,
                        std::vector<FFLD::Object>& objects_ )
{
  xmlDoc * doc = xmlParseFile(filename.c_str());
	if (doc == NULL) {
		std::cerr << "Could not open " << filename << std::endl;
		return -1;
	}
	
	xmlNodePtr cur = xmlDocGetRootElement(doc);
	if (cur == NULL) {
		xmlFreeDoc(doc);
		std::cerr << "Could not open " << filename << std::endl;
		return -1;
	}
	if (xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("annotation"))) {
		xmlFreeDoc(doc);
		std::cerr << "Could not open " << filename << std::endl;
		return -1;
	}	
	
	cur = cur->xmlChildrenNode;
	
	while (cur != NULL)
	{
    if (!xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("size"))) {
			xmlNodePtr cur2 = cur->xmlChildrenNode;
			
			while (cur2 != NULL) {
				if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("width")))
					width_ = content<int>(cur2);
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("height")))
					height_ = content<int>(cur2);
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("depth")))
					depth_ = content<int>(cur2);
				
				cur2 = cur2->next;
			}
		}
		else if (!xmlStrcmp(cur->name, reinterpret_cast<const xmlChar *>("object"))) {
			objects_.push_back(FFLD::Object());
			
			xmlNodePtr cur2 = cur->xmlChildrenNode;
			
			while (cur2 != NULL) {
				if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("name")))
				  objects_.back().setName( FFLD::Object::StringToName( content<std::string>(cur2).c_str() ) );
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("pose")))
				  objects_.back().setPose( FFLD::Object::StringToPose( content<std::string>(cur2).c_str() ) );
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("truncated")))
					objects_.back().setTruncated(content<bool>(cur2));
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("difficult")))
					objects_.back().setDifficult(content<bool>(cur2));
				else if (!xmlStrcmp(cur2->name, reinterpret_cast<const xmlChar *>("bndbox"))) {
					FFLD::Rectangle bndbox;
					
					xmlNodePtr cur3 = cur2->xmlChildrenNode;
					
					while (cur3 != NULL) {
						if (!xmlStrcmp(cur3->name, reinterpret_cast<const xmlChar *>("xmin")))
							bndbox.setX(content<int>(cur3));
						else if (!xmlStrcmp(cur3->name, reinterpret_cast<const xmlChar *>("ymin")))
							bndbox.setY(content<int>(cur3));
						else if (!xmlStrcmp(cur3->name, reinterpret_cast<const xmlChar *>("xmax")))
							bndbox.setWidth(content<int>(cur3));
						else if (!xmlStrcmp(cur3->name, reinterpret_cast<const xmlChar *>("ymax")))
							bndbox.setHeight(content<int>(cur3));
						
						cur3 = cur3->next;
					}
					
					// Only set the bounding box if all values have been assigned
					if (bndbox.x() && bndbox.y() && bndbox.width() && bndbox.height()) {
						bndbox.setX(bndbox.x() - 1);
						bndbox.setY(bndbox.y() - 1);
						bndbox.setWidth(bndbox.width() - bndbox.x());
						bndbox.setHeight(bndbox.height() - bndbox.y());
						objects_.back().setBndbox(bndbox);
					}
				}
				
				cur2 = cur2->next;
			}
		}
		cur = cur->next;
	}
	xmlFreeDoc(doc);
	return 0;
}






// Train a mixture model
int main(int argc, char **argv)
{
	// Default parameters
	double C = 0.002;
	int nbDatamine = 10;
	int interval = 5;
	double J = 2.0;
	int nbRelabel = 8;
	std::string model;
	FFLD::Object::Name name = FFLD::Object::UNKNOWN; // TODO: Fix this
	int padding = 6;
	std::string result("model.txt");
	int seed = static_cast<int>(time(0));
	double overlap = 0.7;
	int nbComponents = 3;
	int nbNegativeScenes = -1;
	std::string ann_dir(".");
	
	// Parse the parameters
	CSimpleOpt args(argc, argv, SOptions);
  
	while (args.Next()) {
		if (args.LastError() == SO_SUCCESS) {
			if (args.OptionId() == OPT_C) {
				C = atof(args.OptionArg());
				
				if (C <= 0) {
					showUsage();
					std::cerr << "\nInvalid C arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_DATAMINE) {
				nbDatamine = atoi(args.OptionArg());
				
				if (nbDatamine <= 0) {
					showUsage();
					std::cerr << "\nInvalid datamine arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_INTERVAL) {
				interval = atoi(args.OptionArg());
				
				if (interval <= 0) {
					showUsage();
					std::cerr << "\nInvalid interval arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_HELP) {
				showUsage();
				return 0;
			}
			else if (args.OptionId() == OPT_J) {
				J = atof(args.OptionArg());
				
				if (J <= 0) {
					showUsage();
					std::cerr << "\nInvalid J arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_RELABEL) {
				nbRelabel = atoi(args.OptionArg());
				
				if (nbRelabel <= 0) {
					showUsage();
					std::cerr << "\nInvalid relabel arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_MODEL) {
				model = args.OptionArg();
			}
			else if (args.OptionId() == OPT_NAME) {
			  std::string arg = args.OptionArg();
			  std::transform(arg.begin(), arg.end(), arg.begin(), static_cast<int (*)(int)>(tolower));
			  name = FFLD::Object::StringToName( arg.c_str() );
			}
			else if (args.OptionId() == OPT_PADDING) {
				padding = std::atoi(args.OptionArg());
				
				if (padding <= 1) {
					showUsage();
					std::cerr << "\nInvalid padding arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_RESULT) {
				result = args.OptionArg();
			}
			else if (args.OptionId() == OPT_SEED) {
				seed = atoi(args.OptionArg());
			}
			else if (args.OptionId() == OPT_OVERLAP) {
				overlap = atof(args.OptionArg());
				
				if ((overlap <= 0.0) || (overlap >= 1.0)) {
					showUsage();
					std::cerr << "\nInvalid overlap arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_NB_COMP) {
				nbComponents = atoi(args.OptionArg());
				
				if (nbComponents <= 0) {
					showUsage();
					std::cerr << "\nInvalid nb-components arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_NB_NEG) {
				nbNegativeScenes = atoi(args.OptionArg());
				
				if (nbNegativeScenes < 0) {
					showUsage();
					std::cerr << "\nInvalid nb-negatives arg " << args.OptionArg() << std::endl;
					return -1;
				}
			}
			else if (args.OptionId() == OPT_AN_DIR) {
			  ann_dir = args.OptionArg();
			}
		}
		else {
			showUsage();
			std::cerr << "\nUnknown option " << args.OptionText() << std::endl;
			return -1;
		}
	}
	
	if (!args.FileCount()) {
		showUsage();
		std::cerr << "\nNo image directory provided" << std::endl;
		return -1;
	}
	else if (args.FileCount() > 1) {
		showUsage();
		std::cerr << "\nMore than one image directory provided" << std::endl;
		return -1;
	}
  const std::string image_dir(args.File(0));
  //std::string ann_dir( args.File(0) );
  
	// Load all the scenes
	int maxRows = 0;
	int maxCols = 0;
	
	std::vector<FFLD::Scene> scenes;
	
	//std::cout << image_dir << std::endl;
	//std::cout << FFLD::Object::StringToName( "car" ) << std::endl;
	//std::cout << FFLD::Object::StringToName( "polaris" ) << std::endl;
	//std::cout << FFLD::Object::StringToPose( "front" ) << std::endl;
	//std::cout << FFLD::Object::StringToPose( "Rear" ) << std::endl;
	
	// Traverse img_dir and look for annotations
	boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
	for ( boost::filesystem::directory_iterator itr( image_dir );
        itr != end_itr; ++itr )
  {
    std::string img_path( itr->path().string() );
    std::string img_name( itr->path().stem().string() );
    std::string ann_name( ann_dir + "/" + img_name + ".xml" );
    
    // Open annotation if exists
    FFLD::Scene scene;
    if( boost::filesystem::exists( ann_name ) )
    {
      int width;
	    int height;
	    int depth;
	    std::vector<FFLD::Object> objects;
	    if( parseAnnotationXML( ann_name, width, height, depth, objects ) == 0 )
        scene = FFLD::Scene( width, height, depth, img_path, objects );
      else
        continue;
    }
    else
    {
      // Else load the image and create an empty scene
      FFLD::JPEGImage im( img_path ); // TODO: Fix this
      scene = FFLD::Scene( im.width(), im.height(), im.depth(), img_path, std::vector<FFLD::Object>() );
    }
    
		if (scene.empty())
			continue;    
		
		bool positive = false;
		bool negative = true;
		

		for (int i = 0; i < scene.objects().size(); ++i) {
			if (scene.objects()[i].name() == name) {
				negative = false;
				
				if (!scene.objects()[i].difficult())
					positive = true;
			}
		}
		
		if (positive || (negative && nbNegativeScenes)) {
			scenes.push_back(scene);
			
			maxRows = std::max(maxRows, (scene.height() + 3) / 4 + padding);
			maxCols = std::max(maxCols, (scene.width() + 3) / 4 + padding);
			
			if (negative)
				--nbNegativeScenes;
		}
  }

	if (scenes.empty()) {
		showUsage();
		std::cerr << "\nNo scenes found in image_dir " << image_dir << std::endl;
		return -1;
	}
	
	
	// Random numbers
	srand(seed);
	srand48(seed);
	
	// Initialize the Patchwork class
	if (!FFLD::Patchwork::InitFFTW((maxRows + 15) & ~15, (maxCols + 15) & ~15)) {
		std::cerr << "Error initializing the FFTW library" << std::endl;
		return - 1;
	}
	
	// The mixture to train
	FFLD::Mixture mixture(nbComponents, scenes, name);		

	if (mixture.empty()) {
		std::cerr << "Error initializing the mixture model" << std::endl;
		return -1;
	}
	
	// Try to open the mixture
	if (!model.empty()) {
		std::ifstream in(model.c_str(), std::ios::binary);
		
		if (!in.is_open()) {
			showUsage();
			std::cerr << "\nInvalid model file " << model << std::endl;
			return -1;
		}
		
		in >> mixture;
		
		if (mixture.empty()) {
			showUsage();
			std::cerr << "\nInvalid model file " << model << std::endl;
			return -1;
		}
	}
	
	if (model.empty())
		mixture.train(scenes, name, padding, padding, interval, nbRelabel / 2, nbDatamine, 24000, C, J, overlap);	
	
	if (mixture.models()[0].parts().size() == 1)
		mixture.initializeParts(8, std::make_pair(6, 6));
	
	mixture.train(scenes, name, padding, padding, interval, nbRelabel, nbDatamine, 24000, C, J, overlap);
	
	// Try to open the result file
	std::ofstream out(result.c_str(), std::ios::binary);
	
	if (!out.is_open()) {
		showUsage();
		std::cerr << "\nInvalid result file " << result << std::endl;
		std::cout << mixture << std::endl; // Print the mixture as a last resort
		return -1;
	}
	
	out << mixture;
	
	return EXIT_SUCCESS;
}



