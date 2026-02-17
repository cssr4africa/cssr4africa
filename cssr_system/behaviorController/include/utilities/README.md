<div align="center">
  <h1>Culture Knowledge Base Helper Class</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The utilities ROS package provides stand-alone software, such as the C++ helper classes that provide access to the culture knowledge base and the environment knowledge base. These classes are used by the ROS nodes that need to access this knowledge; see [Deliverable D3.1 System Architecture](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.1.pdf). 

At present, there are two helper classes: CultureKnowledgeBase and EnvironmentKnowledgeBase. These are C++  classes to read the culture and environment knowledge base files,  store the knowledge in binary search tree dictionary data structures, and make the knowledge accessible through public access methods.

## Documentation
[Deliverable D5.4.1 Cultural Knowledge Ontology & Culture Knowledge Base](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.1.pdf) provides a
 a detailed explanation of the knowledge base and the helper class. It also provides an example of how to use the class to retrieve knowledge by instantiating the class and invoking two public methods.
 
 [Deliverable D5.4.2 Robot Mission Language](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.2.pdf) provides a
 a detailed explanation of the knowledge base and the helper class. It also provides an example of how to use the class to retrieve knowledge by instantiating the class and invoking three public methods.
 
## Running Tests
Do the following to launch the example application to test the helper classes.
```sh
roslaunch utilities cultureKnowledgeBaseExample.launch
roslaunch utilities environmentKnowledgeBaseExample.launch
```
 or 
 
```sh
rosrun utilities cultureKnowledgeBaseExample
rosrun utilities environmentKnowledgeBaseExample
```
## Support
For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:aakinade@andrew.cmu.edu">aakinade@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec) 
Inclusive Digital Transformation Research Grant Programme

Date:   2025-03-03
