import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrier_gate")

    # Colors
    mat_housing = Material(name="mat_housing", color=(0.85, 0.45, 0.1)) # Orange-ish yellow
    mat_dark = Material(name="mat_dark", color=(0.2, 0.2, 0.2))
    mat_metal = Material(name="mat_metal", color=(0.7, 0.7, 0.7))
    mat_boom = Material(name="mat_boom", color=(0.95, 0.95, 0.95))
    mat_stripe = Material(name="mat_stripe", color=(0.85, 0.15, 0.15))
    mat_green = Material(name="mat_green", color=(0.1, 0.8, 0.1))
    mat_red = Material(name="mat_red", color=(0.8, 0.1, 0.1))
    mat_light = Material(name="mat_light", color=(1.0, 0.8, 0.1))

    # 1. Housing (Base)
    housing = model.part("housing")
    
    # Base plate
    housing.visual(
        Box((0.45, 0.45, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_plate",
        material=mat_dark,
    )
    
    # Main body
    housing_shape = (
        cq.Workplane("XY")
        .box(0.35, 0.35, 0.95, centered=(True, True, False))
        .edges("|Z").fillet(0.05)
    )
    housing.visual(
        mesh_from_cadquery(housing_shape, "housing_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="main_body",
        material=mat_housing,
    )
    
    # Lid
    lid_shape = (
        cq.Workplane("XY")
        .rect(0.38, 0.38)
        .extrude(0.05)
        .edges(">Z").chamfer(0.02)
    )
    housing.visual(
        mesh_from_cadquery(lid_shape, "housing_lid"),
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        name="lid",
        material=mat_dark,
    )
    
    # Pivot shaft (extends from X=0.175 to X=0.25)
    housing.visual(
        Cylinder(radius=0.06, length=0.075),
        origin=Origin(xyz=(0.2125, 0.0, 0.85), rpy=(0.0, math.pi / 2, 0.0)),
        name="pivot_shaft",
        material=mat_metal,
    )
    
    # Control box on the -Y face
    housing.visual(
        Box((0.2, 0.05, 0.3)),
        origin=Origin(xyz=(0.0, -0.2, 0.6)),
        name="control_box",
        material=mat_metal,
    )
    
    # Buttons on the control box
    housing.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(0.05, -0.235, 0.65), rpy=(math.pi / 2, 0.0, 0.0)),
        name="btn_green",
        material=mat_green,
    )
    housing.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(-0.05, -0.235, 0.65), rpy=(math.pi / 2, 0.0, 0.0)),
        name="btn_red",
        material=mat_red,
    )
    
    # Vent grille on the +Y face
    housing.visual(
        Box((0.2, 0.02, 0.2)),
        origin=Origin(xyz=(0.0, 0.185, 0.3)),
        name="vent_grille",
        material=mat_dark,
    )
    
    # Warning light base on top of the lid
    housing.visual(
        Cylinder(radius=0.08, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        name="light_base",
        material=mat_dark,
    )
    
    # Warning light dome
    housing.visual(
        Sphere(radius=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        name="warning_light",
        material=mat_light,
    )

    # 2. Arm
    arm = model.part("arm")
    
    # Arm mount (clamp around the pivot axis)
    arm.visual(
        Cylinder(radius=0.1, length=0.1),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="arm_mount",
        material=mat_dark,
    )
    
    # Boom (extends from Y=0.0 to Y=3.5)
    boom_shape = (
        cq.Workplane("XZ")
        .polygon(8, 0.1)
        .extrude(-3.5) # Y goes from 0 to 3.5
    )
    arm.visual(
        mesh_from_cadquery(boom_shape, "boom_tube"),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        name="boom",
        material=mat_boom,
    )
    
    # Red stripes on the boom
    stripe_shape = (
        cq.Workplane("XZ")
        .polygon(8, 0.105)
        .extrude(-0.2)
    )
    for i in range(1, 7):
        y_pos = i * 0.5
        arm.visual(
            mesh_from_cadquery(stripe_shape, f"stripe_{i}"),
            origin=Origin(xyz=(0.05, y_pos, 0.0)),
            name=f"stripe_{i}",
            material=mat_stripe,
        )
        
    # Counterweight (extends from Y=0.0 to Y=-0.4)
    arm.visual(
        Box((0.08, 0.4, 0.15)),
        origin=Origin(xyz=(0.05, -0.2, 0.0)),
        name="counterweight",
        material=mat_dark,
    )

    # Articulation
    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.25, 0.0, 0.85)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=math.pi / 2),
    )

    # 3. Catch Post
    catch_post = model.part("catch_post")
    
    catch_post.visual(
        Box((0.2, 0.2, 0.05)),
        origin=Origin(xyz=(0.3, 3.4, 0.025)),
        name="post_base",
        material=mat_dark,
    )
    
    catch_post.visual(
        Box((0.08, 0.08, 0.73)),
        origin=Origin(xyz=(0.3, 3.4, 0.415)),
        name="post_body",
        material=mat_housing,
    )
    
    catch_post.visual(
        Box((0.16, 0.08, 0.02)),
        origin=Origin(xyz=(0.3, 3.4, 0.79)),
        name="post_bracket_base",
        material=mat_metal,
    )
    
    catch_post.visual(
        Box((0.02, 0.08, 0.05)),
        origin=Origin(xyz=(0.23, 3.4, 0.825)),
        name="post_fork_left",
        material=mat_metal,
    )
    
    catch_post.visual(
        Box((0.02, 0.08, 0.05)),
        origin=Origin(xyz=(0.37, 3.4, 0.825)),
        name="post_fork_right",
        material=mat_metal,
    )

    model.articulation(
        "housing_to_catch_post",
        ArticulationType.FIXED,
        parent=housing,
        child=catch_post,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    arm = object_model.get_part("arm")
    catch_post = object_model.get_part("catch_post")
    joint = object_model.get_articulation("housing_to_arm")
    
    ctx.allow_isolated_part(catch_post, reason="Catch post is a separate physical structure anchored to the ground.")
    
    # Check that arm mount contacts the pivot shaft
    ctx.expect_contact(arm, housing, elem_a="arm_mount", elem_b="pivot_shaft", name="arm_mount_contacts_shaft")
    
    # Check that arm clears the housing body
    ctx.expect_gap(arm, housing, axis="x", positive_elem="arm_mount", negative_elem="main_body", min_gap=0.0, name="arm_clears_housing")
    
    # Check that boom rests on the catch post
    ctx.expect_contact(arm, catch_post, elem_a="boom", elem_b="post_bracket_base", name="boom_rests_on_catch_post", contact_tol=0.001)
    
    # Check that arm opens upwards
    with ctx.pose({joint: math.pi / 2}):
        boom_pos = ctx.part_element_world_aabb(arm, elem="boom")
        if boom_pos is not None:
            # Boom should be mostly above the pivot (Z=0.85) when open
            boom_min_z = boom_pos[0][2]
            ctx.check("boom_opens_upward", boom_min_z > 0.8, details=f"boom min Z is {boom_min_z}")

    return ctx.report()


object_model = build_object_model()