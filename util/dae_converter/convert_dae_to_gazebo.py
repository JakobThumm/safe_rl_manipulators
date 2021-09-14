import argparse
import os
import numpy as np
import fileinput

import xml.etree.ElementTree as ET

def main():
    parser = argparse.ArgumentParser()
    def str2bool(v):
        if isinstance(v, bool):
            return v
        if v.lower() in ('yes', 'true', 't', 'y', '1', 'True'):
            return True
        elif v.lower() in ('no', 'false', 'f', 'n', '0', 'False'):
            return False
        else:
            raise argparse.ArgumentTypeError('Boolean value expected.')

    parser.add_argument('--dae_path', help='Path to the Collada dae file you would like to convert to Gazebo format.')
    parser.add_argument('--fix_dae_file', type=str2bool, default=False, help='Fix the given Collada dae file to fit the gazebo format.')
    parser.add_argument('--fix_dae_colors', type=str2bool, default=False, help='Fix the colors of the Collada dae file.')
    parser.add_argument('--bvh_path', default="", help='Optional path to a bvh file that you can convert as well.')

    args = parser.parse_args()
    dae_path = args.dae_path
    fix_dae_file = args.fix_dae_file
    fix_dae_colors = args.fix_dae_colors
    bvh_path = args.bvh_path

    # Check dae path for ending 
    dae_filename, dae_ending = os.path.splitext(dae_path)
    assert dae_ending == ".dae", "--dae_path is not a dae file!"
    if bvh_path != "":
        bvh_filename, bvh_ending = os.path.splitext(bvh_path)
        assert bvh_ending == ".bvh", "--bvh_path is not a bvh file!"
    # parse an xml file by name
    # Define namespace
    namespaces = {"": "http://www.collada.org/2005/11/COLLADASchema"}
    ET.register_namespace("", namespaces[""])
    namespaces["default"] = namespaces[""]
    dae_tree = ET.parse(dae_path)
    root = dae_tree.getroot()
    # Find animation and library_visual_scenes nodes
    animation_node = root.find("default:library_animations", namespaces)
    assert animation_node is not None, "library_animations node not found!"
    library_visual_scenes_node = root.find("default:library_visual_scenes", namespaces)
    assert library_visual_scenes_node is not None, "library_visual_scenes node not found!"

    # Extract the transformation matrix from the library_visual_scenes first node
    visual_scene_node = library_visual_scenes_node[0]
    transformation_matrix, armature_name, root_bone = extract_transform_and_armature_name(visual_scene_node, namespaces)
    if fix_dae_file:
        # Apply transformation matrix to visual scene node and translation of animation if selected
        recursively_apply_transformation(root_bone, transformation_matrix, namespaces)
        apply_transformation_to_animation(animation_node, transformation_matrix, namespaces)
    if fix_dae_colors:
        fix_colors(root, namespaces)
    if fix_dae_file or fix_dae_colors:
        # Write fixed dae file
        output_dae_file = dae_filename + "_gazebo_conform" + dae_ending
        with open(output_dae_file, 'wb') as f:
            dae_tree.write(f)
        ## Find and replace armature_name + "_" in whole document 
        # Gazebo DAE import has issues if the id and name of a node are not identical.
        # Most likely, they use the name tag instead of the id tag in some instances in their import.
        # To fix this, we can find armature_name + "_" in the whole doc and remove it.
        # This is definetly not the cleanest solution. 
        # Going through the whole xml tree and fixing the issue would be better.
        with fileinput.FileInput(output_dae_file, inplace=True) as file:
            for line in file:
                print(line.replace(armature_name + "_", ""), end='')
    

def apply_transformation_to_animation(root_animation_node, transformation_matrix, namespaces):
    """Transform the animation matrices of all animation output nodes.
    ET nodes are mutable objects, so no return needed.
    Args:
        root_animation_node (ET node): Will be changed in this function
        transformation_matrix ((4,4) np.array)
        namespaces (dict): ET xml namespaces
    """
    new_transformation_matrix = np.copy(transformation_matrix)
    all_animations = root_animation_node.findall("default:animation", namespaces)
    is_first = True
    for animation in all_animations:
        animation_name = animation.get("id")
        all_sources = animation.findall("default:source", namespaces)
        for source in all_sources:
            if source.get("id") == animation_name + "-output":
                matrices = extract_matrices_from_source_node(source, namespaces)
                new_matrices_string = ""
                for matrix in matrices:
                    transformed_animation_matrix = rotate_animation(new_transformation_matrix, matrix)
                    new_matrices_string += matrix_to_string(transformed_animation_matrix) + " "
                # Delete last space
                new_matrices_string = new_matrices_string[:-1]
                float_array_node = source.find("default:float_array", namespaces)
                float_array_node.text = new_matrices_string
        if is_first:
            # Create a new transformation matrix with the translation deleted after the first node
            new_transformation_matrix[0:3,3]=0
            is_first = False

def fix_colors(root, namespaces):
    """Fix the ambient color values in the dae effects.
    Args:
        root (ET node): Will be changed in this function
        namespaces (dict): ET xml namespaces
    """
    effect_node = root.find("default:library_effects", namespaces)
    assert effect_node is not None, "library_effects node not found!"
    all_effects = effect_node.findall("default:effect", namespaces)
    assert all_effects is not None, "effect nodes not found!"
    for effect in all_effects:
        profile_node = effect.find("default:profile_COMMON", namespaces)
        assert profile_node is not None, "profile_COMMON node not found!"
        technique_node = profile_node.find("default:technique", namespaces)
        assert technique_node is not None, "technique node not found!"
        phong_node = technique_node.find("default:phong", namespaces)
        assert phong_node is not None, "phong node not found!"
        diffuse_node = phong_node.find("default:diffuse", namespaces)
        assert diffuse_node is not None, "diffuse node not found!"
        diffuse_color = diffuse_node.find("default:color", namespaces)
        assert diffuse_color is not None, "color node not found!"
        ambient_node = phong_node.find("default:ambient", namespaces)
        assert ambient_node is not None, "ambient node not found!"
        ambient_color = ambient_node.find("default:color", namespaces)
        assert ambient_color is not None, "color node not found!"
        ambient_color.text = diffuse_color.text
       

def rotate_animation(transformation_matrix, animation_matrix):
    """Rotate the rotation axis of the animation matrix.
    Args:
        transformation_matrix ((4,4) np.array)
        animation_matrix ((4,4) np.array)
    Returns:
        rotated_animation_matrix
    """
    # Transform translation
    t_a_new = np.matmul(transformation_matrix, animation_matrix)[0:3, 3]
    t_a_new = t_a_new[:,np.newaxis]
    # Transform rotation
    R_a = animation_matrix[0:3, 0:3]
    R_t = transformation_matrix[0:3, 0:3]
    # Convert animation rotation to rotation axis and angle convention
    temp = (R_a[0,0]+R_a[1,1]+R_a[2,2]-1)/2
    if np.abs(temp)>1:
        temp = min(1, max(-1, temp))
        print("WARNING: (R_a[0,0]+R_a[1,1]+R_a[2,2]-1)/2 = {} is out of bound for arccos. \
            Clipping the value to {}".format((R_a[0,0]+R_a[1,1]+R_a[2,2]-1)/2, temp))
    theta = np.arccos(temp)
    # Converting the rotation only makes sense, when the animation rotation angle is not zero
    if theta!=0:
        r = 1/(2*np.sin(theta)) * np.array([[R_a[2,1]-R_a[1,2]], [R_a[0,2]-R_a[2,0]], [R_a[1,0]-R_a[0,1]]])
        norm_r = np.linalg.norm(r)
        # Rotate the rotation axis of the animation
        r_new = np.matmul(R_t, r)
        # Make sure the length of the rotation vector stays the same. (Is that needed?)
        norm_r_new = np.linalg.norm(r_new)
        r_new = norm_r/norm_r_new * r_new
        # Convert to quaternion to avoid singularities
        eta = np.cos(theta/2)
        eps = np.sin(theta/2) * r_new
        # Convert to new rotation matrix
        R_a = np.array([[2*(eta**2 + eps[0,0]**2)-1, 2*(eps[0,0]*eps[1,0]-eta*eps[2,0]), 2*(eps[0,0]*eps[2,0]+eta*eps[1,0])],
                        [2*(eps[0,0]*eps[1,0]+eta*eps[2,0]), 2*(eta**2 + eps[1,0]**2)-1, 2*(eps[1,0]*eps[2,0]-eta*eps[0,0])],
                        [2*(eps[0,0]*eps[2,0]-eta*eps[1,0]), 2*(eps[1,0]*eps[2,0]+eta*eps[0,0]), 2*(eta**2 + eps[2,0]**2)-1]])
    new_animation = np.concatenate((np.concatenate((R_a, t_a_new), axis=1), np.array([0, 0, 0, 1])[np.newaxis, :]), axis=0)
    return new_animation
    

def extract_matrices_from_source_node(source_node, namespaces):
    """Extracts matrix information from a source with float array node.
    <source id="s1">
        <float_array id="id1" count="3*N">1 2 3</float_array>
        <technique_common>
            <accessor source="#id1" count="N" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
    </source>
    alternative: 
    ...
        <technique_common>
          <accessor source="#id1" count="3" stride="16">
            <param name="TRANSFORM" type="float4x4"/>
          </accessor>
        </technique_common>
    Args:
        source_node (ET node)
        namespaces (dict): ET xml namespaces
    Returns:
        matrices, np.array, first case: shape=N,3
                            second case: shape= N,4,4
    """
    float_array_node = source_node.find("default:float_array", namespaces)
    assert float_array_node is not None, "Float array node not found in {}".format(source_node.get("id"))
    accessor_node = source_node.find("default:technique_common", namespaces)[0]
    assert accessor_node is not None, "Accessor node not found in {}".format(source_node.get("id"))
    N = int(accessor_node.get("count"))
    stride = int(accessor_node.get("stride"))
    assert stride==3 or stride==16, "Only strides of 3 or 16 are supported by this function yet."
    floats_list = [float(item) for item in float_array_node.text.split()]
    if stride==3:
        return np.reshape(floats_list, [N,3])
    else:
        return np.reshape(floats_list, [N,4,4])

def recursively_apply_transformation(root_bone, transformation_matrix, namespaces, is_first=True):
    """Apply the transformation matrix to all children of the root bone and all grandchildren etc.
    ET nodes are mutable objects, so no return needed.
    Args:
        root_bone (ET node): Will be changed in this function
        transformation_matrix ((4,4) np.array)
        namespaces (dict): ET xml namespaces
        is_first (Boolean): whether this is the first call of the recursion
    """
    new_transformation_matrix = np.copy(transformation_matrix)
    matrix_node = root_bone.find("default:matrix", namespaces)
    if matrix_node is not None:
        
        # Apply transformation
        matrix = string_to_matrix(matrix_node.text, [4, 4])
        matrix = rotate_animation(new_transformation_matrix, matrix)
        matrix_node.text = matrix_to_string(matrix)
    if is_first:
        # Create a new transformation matrix with the translation deleted after the first node
        new_transformation_matrix[0:3,3]=0
    child_nodes = root_bone.findall("default:node", namespaces)
    for child in child_nodes:
        recursively_apply_transformation(child, new_transformation_matrix, namespaces, False)

def extract_transform_and_armature_name(visual_scene_node, namespaces):
    """Extract the transformation matrix and the animation prefix from the visual_scene_node.
    Args:
        visual_scene_node (ET node)
        namespaces (dict): ET xml namespaces
    Returns:
        transformation_matrix (np.array (4,4))
        armature_name (string)    
        root_bone (ET node): Root bone of armeture in library_visual_scenes
    """
    transformation_matrix = None
    armature_name = None
    root_bone = None
    for node in visual_scene_node:
        armature_name = node.attrib["id"]
        # The armeture node has two children matrix and node. 
        root_bone = node.find("default:node", namespaces)
        if root_bone is not None:
            if root_bone.attrib['type'] == "JOINT":
                matrix_node = node.find("default:matrix", namespaces)
                assert matrix_node is not None, "Matrix node of armature in library_visual_scenes not found. \
                    Please use the matrix option when exporting the DAE file."
                transformation_matrix = string_to_matrix(matrix_node.text, [4, 4])
                return transformation_matrix, armature_name, root_bone
    assert root_bone is not None, "Root bone of armature in library_visual_scenes not found."
    return transformation_matrix, armature_name, root_bone

def string_to_matrix(inp_str, mat_shape = [4, 4]):
    """Converts a string to a numpy array with given shape.
    Args:
        inp_str:    String of floats e.g. "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
        mat_shape:  Shape of the matrix
    Returns:
        np.array of floats
    """
    # Split string into separate floats
    floats_list = [float(item) for item in inp_str.split()]
    assert len(floats_list) == mat_shape[0] * mat_shape[1], "Matrix and string dimensions do not match."
    return np.reshape(floats_list, mat_shape)

def matrix_to_string(inp_mat):
    """Convert a 2D np.array to a 1D string."""
    flat_mat = inp_mat.flatten()
    return_str = ""
    for flt in flat_mat:
        return_str = return_str + str(flt) + " "
    return return_str[:-1]

if __name__ == '__main__':
    main()