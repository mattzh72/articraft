import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
    FanRotorShroud,
    VentGrilleGeometry,
    VentGrilleSleeve,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_box_fan")

    # 1. Stand (Root)
    stand = model.part("stand")
    
    # Base of the U-stand
    stand.visual(
        Box((0.30, 0.12, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="stand_base",
    )
    
    # Left arm
    stand.visual(
        Box((0.02, 0.08, 0.13)),
        origin=Origin(xyz=(-0.14, 0.0, 0.085)),
        name="left_arm",
    )
    # Left hinge cylinder
    stand.visual(
        Cylinder(radius=0.02, length=0.04),
        origin=Origin(xyz=(-0.13, 0.0, 0.15), rpy=(0.0, math.pi / 2, 0.0)),
        name="left_hinge_barrel",
    )
    
    # Right arm
    stand.visual(
        Box((0.02, 0.08, 0.13)),
        origin=Origin(xyz=(0.14, 0.0, 0.085)),
        name="right_arm",
    )
    # Right hinge cylinder
    stand.visual(
        Cylinder(radius=0.02, length=0.04),
        origin=Origin(xyz=(0.13, 0.0, 0.15), rpy=(0.0, math.pi / 2, 0.0)),
        name="right_hinge_barrel",
    )

    # 2. Housing
    housing = model.part("housing")
    
    # Top wall
    housing.visual(
        Box((0.24, 0.08, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        name="housing_top",
    )
    # Bottom wall
    housing.visual(
        Box((0.24, 0.08, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        name="housing_bottom",
    )
    # Left wall
    housing.visual(
        Box((0.01, 0.08, 0.22)),
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
        name="housing_left",
    )
    # Right wall
    housing.visual(
        Box((0.01, 0.08, 0.22)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        name="housing_right",
    )
    
    # Front grille
    front_grille = VentGrilleGeometry(
        (0.24, 0.24),
        frame=0.01,
        face_thickness=0.004,
        duct_depth=0.01,
        slat_pitch=0.015,
        slat_width=0.01,
        slat_angle_deg=30.0,
        sleeve=VentGrilleSleeve(style="none"),
    )
    housing.visual(
        mesh_from_geometry(front_grille, "front_grille"),
        # +Z of VentGrille needs to point to +Y. Rotate around X by -90 deg.
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        name="front_grille",
    )
    
    # Back grille
    back_grille = VentGrilleGeometry(
        (0.24, 0.24),
        frame=0.01,
        face_thickness=0.004,
        duct_depth=0.01,
        slat_pitch=0.015,
        slat_width=0.01,
        slat_angle_deg=-30.0, # opposite angle
        sleeve=VentGrilleSleeve(style="none"),
    )
    housing.visual(
        mesh_from_geometry(back_grille, "back_grille"),
        # +Z of VentGrille needs to point to -Y. Rotate around X by 90 deg.
        origin=Origin(xyz=(0.0, -0.04, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="back_grille",
    )

    # Motor body
    housing.visual(
        Cylinder(radius=0.03, length=0.035),
        origin=Origin(xyz=(0.0, -0.0225, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="motor_body",
    )

    # Housing tilts on horizontal hinges at the ends of the U-stand
    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(1.0, 0.0, 0.0), # Tilts around X axis
        motion_limits=MotionLimits(lower=-math.pi/4, upper=math.pi/4),
    )

    # 3. Fan Blade
    fan_blade = model.part("fan_blade")
    rotor = FanRotorGeometry(
        outer_radius=0.10,
        hub_radius=0.03,
        blade_count=5,
        thickness=0.015,
        blade_pitch_deg=25.0,
        blade_sweep_deg=15.0,
        blade=FanRotorBlade(shape="scimitar", camber=0.1),
        hub=FanRotorHub(style="domed"),
        shroud=FanRotorShroud(thickness=0.002, depth=0.015, clearance=0.002),
    )
    fan_blade.visual(
        mesh_from_geometry(rotor, "rotor"),
        # Rotor spins around local Z. We want it to spin around Y.
        # So rotate by -90 deg around X.
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        name="rotor",
    )
    
    # Fan spins continuously behind the grille
    model.articulation(
        "housing_to_fan_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_blade,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # 4. Control Knob
    knob = model.part("control_knob")
    # Small cylinder for the knob
    knob.visual(
        Cylinder(radius=0.015, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="knob_cap",
    )
    # Short shaft
    knob.visual(
        Cylinder(radius=0.005, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        name="knob_shaft",
    )
    
    # Placed on the front corner of the housing
    # Front face is at Y = 0.04.
    # We want the knob to sit on the face. The knob is oriented along Z by default.
    # We want it to be oriented along Y.
    # So origin = (0.09, 0.04, -0.09) and rotate by -90 around X.
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.09, 0.04, -0.09), rpy=(-math.pi / 2, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0), # After rotation, local Z points along global Y. So spin around local Z.
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    fan_blade = object_model.get_part("fan_blade")
    knob = object_model.get_part("control_knob")
    
    # Allowances
    # 1. Hinge barrels penetrate the housing side walls
    ctx.allow_overlap(
        stand, housing,
        elem_a="left_hinge_barrel", elem_b="housing_left",
        reason="Hinge pin is captured in the housing wall."
    )
    ctx.allow_overlap(
        stand, housing,
        elem_a="right_hinge_barrel", elem_b="housing_right",
        reason="Hinge pin is captured in the housing wall."
    )
    
    # 2. Knob shaft penetrates the front grille
    ctx.allow_overlap(
        knob, housing,
        elem_a="knob_shaft", elem_b="front_grille",
        reason="Knob shaft penetrates the front grille to mount to the internal mechanism."
    )
    
    # 3. Fan blade overlaps the motor body
    ctx.allow_overlap(
        fan_blade, housing,
        elem_a="rotor", elem_b="motor_body",
        reason="Fan hub mounts onto the motor shaft."
    )
    
    # Exact assertions
    # 1. Housing is between the stand arms
    ctx.expect_gap(
        stand, housing,
        axis="x",
        positive_elem="right_arm",
        negative_elem="housing_right",
        min_gap=0.005,
    )
    ctx.expect_gap(
        housing, stand,
        axis="x",
        positive_elem="housing_left",
        negative_elem="left_arm",
        min_gap=0.005,
    )
    
    # 2. Knob sits on the housing
    ctx.expect_contact(
        knob, housing,
        elem_a="knob_cap",
        elem_b="front_grille",
    )
    
    # 3. Fan blade is inside the housing
    ctx.expect_within(
        fan_blade, housing,
        axes="xyz",
        margin=0.0,
    )

    return ctx.report()


object_model = build_object_model()
