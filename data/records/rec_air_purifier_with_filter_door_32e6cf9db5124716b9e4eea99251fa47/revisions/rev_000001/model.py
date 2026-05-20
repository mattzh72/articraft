from __future__ import annotations

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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    WheelGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_air_scrubber")

    body_mat = model.material("blue_powder_coat", color=(0.10, 0.24, 0.34, 1.0))
    dark_mat = model.material("dark_access_panel", color=(0.05, 0.06, 0.065, 1.0))
    steel_mat = model.material("galvanized_steel", color=(0.62, 0.65, 0.66, 1.0))
    rubber_mat = model.material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    filter_a_mat = model.material("coarse_prefilter", color=(0.82, 0.84, 0.80, 1.0))
    filter_b_mat = model.material("blue_media", color=(0.30, 0.56, 0.82, 1.0))
    filter_c_mat = model.material("white_hepa_media", color=(0.92, 0.92, 0.88, 1.0))
    filter_d_mat = model.material("carbon_media", color=(0.08, 0.08, 0.075, 1.0))
    label_mat = model.material("warning_label", color=(0.95, 0.72, 0.12, 1.0))

    housing = model.part("housing")

    # Boxy industrial cabinet: open lower front, louvered upper front, and a
    # shallow sheet-metal shell instead of a solid block.
    width = 0.72
    depth = 0.52
    height = 1.20
    base_z = 0.16
    top_z = base_z + height
    t = 0.030
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    housing.visual(
        Box((t, depth + 0.010, height)),
        origin=Origin(xyz=(-width / 2.0 + t / 2.0, 0.0, base_z + height / 2.0)),
        material=body_mat,
        name="side_panel_0",
    )
    housing.visual(
        Box((t, depth + 0.010, height)),
        origin=Origin(xyz=(width / 2.0 - t / 2.0, 0.0, base_z + height / 2.0)),
        material=body_mat,
        name="side_panel_1",
    )
    housing.visual(
        Box((width, t, height)),
        origin=Origin(xyz=(0.0, rear_y - t / 2.0, base_z + height / 2.0)),
        material=body_mat,
        name="rear_panel",
    )
    housing.visual(
        Box((width, depth, t)),
        origin=Origin(xyz=(0.0, 0.0, top_z - t / 2.0)),
        material=body_mat,
        name="top_cap",
    )
    housing.visual(
        Box((width, depth, t)),
        origin=Origin(xyz=(0.0, 0.0, base_z + t / 2.0)),
        material=body_mat,
        name="bottom_pan",
    )

    # Front frame surrounding the lower access opening and tying into the upper
    # grille bay.
    frame_y = front_y - t / 2.0
    housing.visual(
        Box((0.055, t, 0.70)),
        origin=Origin(xyz=(-width / 2.0 + 0.0275, frame_y, base_z + 0.35)),
        material=body_mat,
        name="front_stile_0",
    )
    housing.visual(
        Box((0.055, t, 0.70)),
        origin=Origin(xyz=(width / 2.0 - 0.0275, frame_y, base_z + 0.35)),
        material=body_mat,
        name="front_stile_1",
    )
    housing.visual(
        Box((width, t, 0.055)),
        origin=Origin(xyz=(0.0, frame_y, base_z + 0.0275)),
        material=body_mat,
        name="lower_sill",
    )
    housing.visual(
        Box((width, t, 0.055)),
        origin=Origin(xyz=(0.0, frame_y, base_z + 0.685)),
        material=body_mat,
        name="middle_rail",
    )
    housing.visual(
        Box((width, t, 0.060)),
        origin=Origin(xyz=(0.0, frame_y, top_z - 0.030)),
        material=body_mat,
        name="upper_header",
    )
    housing.visual(
        Box((0.060, t, 0.44)),
        origin=Origin(xyz=(-width / 2.0 + 0.030, frame_y, base_z + 0.94)),
        material=body_mat,
        name="upper_stile_0",
    )
    housing.visual(
        Box((0.060, t, 0.44)),
        origin=Origin(xyz=(width / 2.0 - 0.030, frame_y, base_z + 0.94)),
        material=body_mat,
        name="upper_stile_1",
    )

    grille_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (0.56, 0.34),
            frame=0.018,
            face_thickness=0.006,
            duct_depth=0.034,
            duct_wall=0.004,
            slat_pitch=0.030,
            slat_width=0.013,
            slat_angle_deg=32.0,
            corner_radius=0.006,
            slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=2),
            frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
            mounts=VentGrilleMounts(style="holes", inset=0.020, hole_diameter=0.006),
            sleeve=VentGrilleSleeve(style="short", depth=0.018, wall=0.004),
        ),
        "front_discharge_grille",
    )
    housing.visual(
        grille_mesh,
        origin=Origin(xyz=(0.0, front_y - 0.026, base_z + 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="front_grille",
    )
    for i, (x, z) in enumerate(((-0.292, 0.835), (0.292, 0.835), (-0.292, 1.045), (0.292, 1.045))):
        housing.visual(
            Box((0.055, 0.050, 0.035)),
            origin=Origin(xyz=(x, frame_y - 0.003, z)),
            material=steel_mat,
            name=f"grille_mount_{i}",
        )

    # Bottom guide rails that retain and support the sliding filter cassette.
    for rail_name, lip_name, x in (
        ("guide_rail_0", "rail_lip_0", -0.22),
        ("guide_rail_1", "rail_lip_1", 0.22),
    ):
        housing.visual(
            Box((0.055, 0.44, 0.030)),
            origin=Origin(xyz=(x, 0.015, 0.2275)),
            material=steel_mat,
            name=rail_name,
        )
        housing.visual(
            Box((0.020, 0.44, 0.030)),
            origin=Origin(xyz=(x + (0.027 if x < 0 else -0.027), 0.015, 0.205)),
            material=steel_mat,
            name=lip_name,
        )

    # Visible exposed hinge-side knuckles fixed to the cabinet frame.
    hinge_x = -0.310
    hinge_y = front_y - 0.055
    parent_knuckles = ((0.09, 0.300), (0.10, 0.525), (0.09, 0.745))
    for i, (length, zc) in enumerate(parent_knuckles):
        housing.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=steel_mat,
            name=f"frame_knuckle_{i}",
        )
        housing.visual(
            Box((0.050, 0.020, length * 0.72)),
            origin=Origin(xyz=(hinge_x - 0.026, hinge_y + 0.020, zc)),
            material=steel_mat,
            name=f"frame_hinge_leaf_{i}",
        )
    housing.visual(
        Cylinder(radius=0.006, length=0.535),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.5225)),
        material=steel_mat,
        name="hinge_pin",
    )

    # Four static caster assemblies, each tied to the bottom pan by a stem and
    # fork. The wheel/tire geometry is aligned on a transverse local X axle.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.052,
            0.032,
            inner_radius=0.034,
            tread=TireTread(style="block", depth=0.004, count=12, land_ratio=0.58),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    wheel_mesh = mesh_from_geometry(WheelGeometry(0.035, 0.030), "caster_wheel")
    caster_positions = (
        (-0.285, -0.195),
        (0.285, -0.195),
        (-0.285, 0.195),
        (0.285, 0.195),
    )
    for i, (x, y) in enumerate(caster_positions):
        housing.visual(
            Box((0.095, 0.080, 0.012)),
            origin=Origin(xyz=(x, y, 0.154)),
            material=steel_mat,
            name=f"caster_plate_{i}",
        )
        housing.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(xyz=(x, y, 0.126)),
            material=steel_mat,
            name=f"caster_stem_{i}",
        )
        housing.visual(
            Box((0.010, 0.026, 0.080)),
            origin=Origin(xyz=(x - 0.028, y, 0.080)),
            material=steel_mat,
            name=f"caster_fork_0_{i}",
        )
        housing.visual(
            Box((0.010, 0.026, 0.080)),
            origin=Origin(xyz=(x + 0.028, y, 0.080)),
            material=steel_mat,
            name=f"caster_fork_1_{i}",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(x, y, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"caster_axle_{i}",
        )
        housing.visual(
            tire_mesh,
            origin=Origin(xyz=(x, y, 0.055)),
            material=rubber_mat,
            name=f"caster_tire_{i}",
        )
        housing.visual(
            wheel_mesh,
            origin=Origin(xyz=(x, y, 0.055)),
            material=steel_mat,
            name=f"caster_wheel_{i}",
        )

    # Hinged lower access door. The child frame is on the vertical hinge axis
    # and the door panel extends to local +X. Positive motion opens outward.
    door = model.part("door")
    door_w = 0.580
    door_h = 0.580
    door_t = 0.028
    door_bottom = 0.230
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.315, 0.007, door_h / 2.0)),
        material=dark_mat,
        name="door_panel",
    )
    door.visual(
        Box((0.500, 0.006, 0.035)),
        origin=Origin(xyz=(0.330, -0.008, 0.080)),
        material=label_mat,
        name="lower_label",
    )
    door.visual(
        Box((0.500, 0.006, 0.035)),
        origin=Origin(xyz=(0.330, -0.008, door_h - 0.080)),
        material=label_mat,
        name="upper_label",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.24),
        origin=Origin(xyz=(0.515, -0.060, 0.315)),
        material=steel_mat,
        name="pull_handle_bar",
    )
    for i, z in enumerate((0.225, 0.405)):
        door.visual(
            Cylinder(radius=0.009, length=0.055),
            origin=Origin(xyz=(0.515, -0.032, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"pull_handle_standoff_{i}",
        )
    door.visual(
        Box((0.060, 0.010, 0.030)),
        origin=Origin(xyz=(0.535, -0.008, 0.120)),
        material=steel_mat,
        name="compression_latch",
    )
    door_knuckles = ((0.095, 0.412), (0.095, 0.635))
    for i, (length, zc_world) in enumerate(door_knuckles):
        local_z = zc_world - door_bottom
        door.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=steel_mat,
            name=f"door_knuckle_{i}",
        )
        door.visual(
            Box((0.060, 0.010, length * 0.70)),
            origin=Origin(xyz=(0.032, 0.004, local_z)),
            material=steel_mat,
            name=f"door_hinge_leaf_{i}",
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=45.0, velocity=1.2),
    )

    # Sliding four-stage filter cassette. The rack rides on the guide rails and
    # the four filter stages are visibly layered front-to-back inside the rack.
    stack = model.part("filter_stack")
    stack_origin = (0.0, 0.010, 0.550)
    stack.visual(
        Box((0.580, 0.405, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.295)),
        material=steel_mat,
        name="bottom_sled",
    )
    for runner_name, x in (("runner_0", -0.220), ("runner_1", 0.220)):
        stack.visual(
            Box((0.050, 0.390, 0.025)),
            origin=Origin(xyz=(x, 0.0, -0.295)),
            material=steel_mat,
            name=runner_name,
        )
    for i, x in enumerate((-0.270, 0.270)):
        stack.visual(
            Box((0.030, 0.405, 0.540)),
            origin=Origin(xyz=(x, 0.0, -0.015)),
            material=steel_mat,
            name=f"rack_side_{i}",
        )
    stack.visual(
        Box((0.570, 0.030, 0.540)),
        origin=Origin(xyz=(0.0, -0.205, -0.015)),
        material=steel_mat,
        name="front_rack_frame",
    )
    stack.visual(
        Box((0.570, 0.030, 0.540)),
        origin=Origin(xyz=(0.0, 0.205, -0.015)),
        material=steel_mat,
        name="rear_rack_frame",
    )
    stack.visual(
        Box((0.570, 0.405, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=steel_mat,
        name="top_filter_frame",
    )
    stage_specs = (
        ("stage_0", -0.135, filter_a_mat),
        ("stage_1", -0.045, filter_b_mat),
        ("stage_2", 0.045, filter_c_mat),
        ("stage_3", 0.135, filter_d_mat),
    )
    for stage_name, y, mat in stage_specs:
        stack.visual(
            Box((0.510, 0.045, 0.455)),
            origin=Origin(xyz=(0.0, y, -0.020)),
            material=mat,
            name=stage_name,
        )
        # Raised pleat ribs on each filter face make the stages read as media,
        # not plain colored blocks.
        for rib_i, x in enumerate((-0.205, -0.145, -0.085, -0.025, 0.035, 0.095, 0.155, 0.215)):
            stack.visual(
                Box((0.010, 0.010, 0.420)),
                origin=Origin(xyz=(x, y - 0.0275, -0.020)),
                material=steel_mat if stage_name == "stage_3" else dark_mat,
                name=f"{stage_name}_pleat_{rib_i}",
            )
    stack.visual(
        Box((0.160, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, -0.231, -0.020)),
        material=steel_mat,
        name="cassette_pull",
    )

    model.articulation(
        "housing_to_filter_stack",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=stack,
        origin=Origin(xyz=stack_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.340, effort=120.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    stack = object_model.get_part("filter_stack")
    door_joint = object_model.get_articulation("housing_to_door")
    stack_joint = object_model.get_articulation("housing_to_filter_stack")

    for knuckle in ("door_knuckle_0", "door_knuckle_1"):
        ctx.allow_overlap(
            housing,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The fixed hinge pin is intentionally captured through the door knuckle bore.",
        )
        ctx.expect_within(
            housing,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle,
            margin=0.001,
            name=f"hinge pin is centered inside {knuckle}",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.080,
            name=f"hinge pin spans {knuckle}",
        )

    ctx.expect_gap(
        housing,
        door,
        axis="y",
        positive_elem="front_stile_1",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.014,
        name="closed door sits just proud of the front frame",
    )
    ctx.expect_gap(
        stack,
        housing,
        axis="z",
        positive_elem="runner_0",
        negative_elem="guide_rail_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="filter cassette rests on the left guide rail",
    )
    ctx.expect_overlap(
        stack,
        housing,
        axes="y",
        elem_a="runner_0",
        elem_b="guide_rail_0",
        min_overlap=0.30,
        name="collapsed filter runner is deeply retained",
    )
    for stage_name in ("stage_0", "stage_1", "stage_2", "stage_3"):
        ctx.expect_within(
            stack,
            stack,
            axes="xz",
            inner_elem=stage_name,
            outer_elem="front_rack_frame",
            margin=0.010,
            name=f"{stage_name} is captured by the cassette frame",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.45}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door opens outward about its left vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.25,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_pos = ctx.part_world_position(stack)
    with ctx.pose({door_joint: 1.55, stack_joint: 0.340}):
        extended_pos = ctx.part_world_position(stack)
        ctx.expect_overlap(
            stack,
            housing,
            axes="y",
            elem_a="runner_0",
            elem_b="guide_rail_0",
            min_overlap=0.070,
            name="extended filter stack remains retained on rails",
        )
        ctx.expect_gap(
            stack,
            housing,
            axis="z",
            positive_elem="runner_1",
            negative_elem="guide_rail_1",
            max_gap=0.002,
            max_penetration=0.0,
            name="extended cassette still rides on the right guide rail",
        )
    ctx.check(
        "filter stack slides out through the open front",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
