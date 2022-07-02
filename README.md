# Generating Predicate Suggestions based on the Space of Plans. An Example of Planning with Preferences
Authors: Gerard Canal, Carme Torras, Guillem Alenyà

Task planning in human-robot environments tends to be particularly complex as it involves additional uncertainty introduced by the human user. Several plans, entailing few or various differences, can be obtained to solve the same given task. To choose among them, the usual least-cost plan criteria is not necessarily the best option, because here, human constraints and preferences come into play. Knowing these user preferences is very valuable to select an appropriate plan, but the preference values are usually hard to obtain. 
In this context, we propose the Space-of-Plans-based Suggestions (SoPS) algorithms that can provide suggestions for some planning predicates, which are used to define the state of the environment in a task planning problem where actions modify the predicates. We denote these predicates as suggestible predicates , of which user preferences are a particular case. The first algorithm is able to analyze the potential effect of the unknown predicates and provide suggestions to values for these unknown predicates that may produce better plans. The second algorithm is able to suggest changes to already known values that potentially improve the obtained reward. The proposed approach utilizes a Space of Plans Tree structure to represent a subset of the space of plans. The tree is traversed to find the predicates and the values that would most increase the reward, and output them as a suggestion to the user. Our exhaustive evaluation using three preference-based assistive robotics domains shows how the proposed algorithms can improve task performance by suggesting the most effective predicate values first.

## Citation
If you use our work, please cite us as:

```
@article{Canal_UMUAI2022,
    author = "Gerard Canal and Carme Torras and Guillem Alenyà",
    title = "{Generating predicate suggestions based on the space of plans: an example of planning with preferences}",
    journal = "User Modeling and User-Adapted Interaction",
    publisher = "Springer Nature",
    year = "2022",
    doi = "10.1007/s11257-022-09327-w"
}
```
