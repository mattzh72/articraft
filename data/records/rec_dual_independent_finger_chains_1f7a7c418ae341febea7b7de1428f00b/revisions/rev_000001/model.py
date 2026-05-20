import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_link(length: float, width: float, gap: float, cheek_thickness: float, tongue_width: float) -> cq.Workplane:
    radius = width / 2.0
    
    # Tongue at Y=0
    tongue_cyl = cq.Workplane("YZ").cylinder(tongue_width, radius)
    tongue_box = cq.Workplane("XY").center(0, length/4 + 0.001).box(tongue_width, length/2 + 0.002, width)
    tongue = tongue_cyl.union(tongue_box)
    
    # Cheeks at Y=length
    offset = gap/2 + cheek_thickness/2
    l_cyl = cq.Workplane("YZ").center(length, 0).workplane(offset=-offset).cylinder(cheek_thickness, radius)
    l_box = cq.Workplane("XY").center(-offset, length*0.75 - 0.001).box(cheek_thickness, length/2 + 0.002, width)
    
    r_cyl = cq.Workplane("YZ").center(length, 0).workplane(offset=offset).cylinder(cheek_thickness, radius)
    r_box = cq.Workplane("XY").center(offset, length*0.75 - 0.001).box(cheek_thickness, length/2 + 0.002, width)
    
    return tongue.union(l_cyl).union(l_box).union(r_cyl).union(r_box)

def make_distal_link(length: float, width: float, tongue_width: float) -> cq.Workplane:
    radius = width / 2.0
    tongue_cyl = cq.Workplane("YZ").cylinder(tongue_width, radius)
    tongue_box = cq.Workplane("XY").center(0, length/2).box(tongue_width, length, width)
    tip_cyl = cq.Workplane("YZ").center(length, 0).cylinder(tongue_width, radius)
    return tongue_cyl.union(tongue_box).union(tip_cyl)

def make_palm_plate() -> cq.Workplane:
    # Plate
    plate = cq.Workplane("XY").center(0, -0.02).workplane(offset=-0.01).box(0.10, 0.08, 0.02)
    
    # Mounts
    width = 0.02
    gap = 0.030
    cheek_thickness = 0.005
    
    def add_mount(base: cq.Workplane, center_x: float) -> cq.Workplane:
        offset = gap/2 + cheek_thickness/2
        l_cyl = cq.Workplane("YZ").center(0, 0.015).workplane(offset=center_x - offset).cylinder(cheek_thickness, width/2)
        r_cyl = cq.Workplane("YZ").center(0, 0.015).workplane(offset=center_x + offset).cylinder(cheek_thickness, width/2)
        l_box = cq.Workplane("XY").center(center_x - offset, 0).workplane(offset=0.0075).box(cheek_thickness, width, 0.015)
        r_box = cq.Workplane("XY").center(center_x + offset, 0).workplane(offset=0.0075).box(cheek_thickness, width, 0.015)
        return base.union(l_cyl).union(r_cyl).union(l_box).union(r_box)
        
    plate = add_mount(plate, -0.03)
    plate = add_mount(plate, 0.03)
    return plate

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_dual_finger")
    
    # Materials
    base_mat = model.material("base_mat", rgba=(0.2, 0.2, 0.2, 1.0))
    link1_mat = model.material("link1_mat", rgba=(0.8, 0.3, 0.3, 1.0))
    link2_mat = model.material("link2_mat", rgba=(0.3, 0.8, 0.3, 1.0))
    link3_mat = model.material("link3_mat", rgba=(0.3, 0.3, 0.8, 1.0))

    # Palm Plate
    palm = model.part("palm_plate")
    palm.visual(mesh_from_cadquery(make_palm_plate(), "palm_plate_mesh"), material=base_mat)
    
    # Link geometries
    # Proximal
    prox_geom = make_link(length=0.06, width=0.02, gap=0.020, cheek_thickness=0.005, tongue_width=0.030)
    # Intermediate
    inter_geom = make_link(length=0.04, width=0.016, gap=0.012, cheek_thickness=0.004, tongue_width=0.020)
    # Distal
    distal_geom = make_distal_link(length=0.03, width=0.012, tongue_width=0.012)
    
    # Left Finger
    l_prox = model.part("left_proximal")
    l_prox.visual(mesh_from_cadquery(prox_geom, "left_prox_mesh"), material=link1_mat)
    
    l_inter = model.part("left_intermediate")
    l_inter.visual(mesh_from_cadquery(inter_geom, "left_inter_mesh"), material=link2_mat)
    
    l_distal = model.part("left_distal")
    l_distal.visual(mesh_from_cadquery(distal_geom, "left_distal_mesh"), material=link3_mat)
    
    # Right Finger
    r_prox = model.part("right_proximal")
    r_prox.visual(mesh_from_cadquery(prox_geom, "right_prox_mesh"), material=link1_mat)
    
    r_inter = model.part("right_intermediate")
    r_inter.visual(mesh_from_cadquery(inter_geom, "right_inter_mesh"), material=link2_mat)
    
    r_distal = model.part("right_distal")
    r_distal.visual(mesh_from_cadquery(distal_geom, "right_distal_mesh"), material=link3_mat)
    
    # Articulations
    limits = MotionLimits(lower=-0.2, upper=1.5, effort=10.0, velocity=5.0)
    
    # Left Finger Joints
    model.articulation(
        "left_joint_1",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=l_prox,
        origin=Origin(xyz=(-0.03, 0.0, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "left_joint_2",
        ArticulationType.REVOLUTE,
        parent=l_prox,
        child=l_inter,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "left_joint_3",
        ArticulationType.REVOLUTE,
        parent=l_inter,
        child=l_distal,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    
    # Right Finger Joints
    model.articulation(
        "right_joint_1",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=r_prox,
        origin=Origin(xyz=(0.03, 0.0, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "right_joint_2",
        ArticulationType.REVOLUTE,
        parent=r_prox,
        child=r_inter,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "right_joint_3",
        ArticulationType.REVOLUTE,
        parent=r_inter,
        child=r_distal,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allow overlap because the tongue and clevis fit exactly with minimal clearance,
    # and the cylindrical joints are coaxial.
    ctx.allow_overlap("palm_plate", "left_proximal", reason="Proximal tongue nested in base clevis")
    ctx.allow_overlap("palm_plate", "right_proximal", reason="Proximal tongue nested in base clevis")
    ctx.allow_overlap("left_proximal", "left_intermediate", reason="Intermediate tongue nested in proximal clevis")
    ctx.allow_overlap("right_proximal", "right_intermediate", reason="Intermediate tongue nested in proximal clevis")
    ctx.allow_overlap("left_intermediate", "left_distal", reason="Distal tongue nested in intermediate clevis")
    ctx.allow_overlap("right_intermediate", "right_distal", reason="Distal tongue nested in intermediate clevis")
    
    # Basic containment checks to ensure they are seated
    ctx.expect_within("left_proximal", "palm_plate", axes="x", margin=0.01)
    ctx.expect_within("right_proximal", "palm_plate", axes="x", margin=0.01)
    
    return ctx.report()

object_model = build_object_model()
