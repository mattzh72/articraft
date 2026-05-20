from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender", assets=ASSETS)

    base_width = 0.230
    base_depth = 0.240
    base_height = 0.082
    top_plate_height = 0.010
    seat_height = 0.008

    seat_top_z = base_height + top_plate_height + seat_height

    jar_outer = 0.145
    jar_wall = 0.006
    jar_inner = jar_outer - 2.0 * jar_wall
    jar_collar_height = 0.022
    jar_wall_height = 0.263
    jar_rim_height = 0.010
    jar_height = jar_collar_height + jar_wall_height + jar_rim_height

    lid_width = 0.154
    lid_depth = 0.170
    lid_thickness = 0.014
    lid_plug_size = 0.128
    lid_plug_depth = 0.012
    hinge_radius = 0.006

    metallic = model.material("metallic", rgba=(0.62, 0.64, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    polycarbonate = model.material("polycarbonate", rgba=(0.82, 0.90, 0.98, 0.28))
    accent = model.material("accent", rgba=(0.30, 0.33, 0.37, 1.0))

    base = model.part("base")
    base.visual(
        Box((base_width, base_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=metallic,
        name="body_shell",
    )
    base.visual(
        Box((0.170, 0.170, top_plate_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height + top_plate_height / 2.0)),
        material=charcoal,
        name="top_plate",
    )
    base.visual(
        Box((0.154, 0.154, seat_height)),
        origin=Origin(xyz=(0.0, 0.0, seat_top_z - seat_height / 2.0)),
        material=dark_rubber,
        name="jar_seat",
    )
    base.visual(
        Box((0.120, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -(base_depth / 2.0) + 0.010, 0.044)),
        material=accent,
        name="control_panel",
    )
    for index, x in enumerate((-0.030, 0.0, 0.030), start=1):
        base.visual(
            Box((0.018, 0.008, 0.008)),
            origin=Origin(xyz=(x, -(base_depth / 2.0) + 0.016, 0.044)),
            material=steel,
            name=f"control_key_{index}",
        )
    for x, y, name in (
        (-0.082, -0.082, "foot_fl"),
        (0.082, -0.082, "foot_fr"),
        (-0.082, 0.082, "foot_rl"),
        (0.082, 0.082, "foot_rr"),
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=dark_rubber,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, base_height + top_plate_height + seat_height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (base_height + top_plate_height + seat_height) / 2.0)),
    )

    jar = model.part("jar")
    jar.visual(
        Box((jar_outer, jar_outer, jar_collar_height)),
        origin=Origin(xyz=(0.0, 0.0, jar_collar_height / 2.0)),
        material=charcoal,
        name="collar",
    )
    jar.visual(
        Box((jar_outer, jar_wall, jar_wall_height)),
        origin=Origin(
            xyz=(0.0, -(jar_outer / 2.0) + jar_wall / 2.0, jar_collar_height + jar_wall_height / 2.0)
        ),
        material=polycarbonate,
        name="front_wall",
    )
    jar.visual(
        Box((jar_outer, jar_wall, jar_wall_height)),
        origin=Origin(
            xyz=(0.0, (jar_outer / 2.0) - jar_wall / 2.0, jar_collar_height + jar_wall_height / 2.0)
        ),
        material=polycarbonate,
        name="rear_wall",
    )
    jar.visual(
        Box((jar_wall, jar_inner, jar_wall_height)),
        origin=Origin(
            xyz=(-(jar_outer / 2.0) + jar_wall / 2.0, 0.0, jar_collar_height + jar_wall_height / 2.0)
        ),
        material=polycarbonate,
        name="left_wall",
    )
    jar.visual(
        Box((jar_wall, jar_inner, jar_wall_height)),
        origin=Origin(
            xyz=((jar_outer / 2.0) - jar_wall / 2.0, 0.0, jar_collar_height + jar_wall_height / 2.0)
        ),
        material=polycarbonate,
        name="right_wall",
    )
    rim_z = jar_collar_height + jar_wall_height + jar_rim_height / 2.0
    jar.visual(
        Box((jar_outer, jar_wall, jar_rim_height)),
        origin=Origin(xyz=(0.0, -(jar_outer / 2.0) + jar_wall / 2.0, rim_z)),
        material=dark_rubber,
        name="front_rim",
    )
    jar.visual(
        Box((jar_outer, jar_wall, jar_rim_height)),
        origin=Origin(xyz=(0.0, (jar_outer / 2.0) - jar_wall / 2.0, rim_z)),
        material=dark_rubber,
        name="rear_rim",
    )
    jar.visual(
        Box((jar_wall, jar_inner, jar_rim_height)),
        origin=Origin(xyz=(-(jar_outer / 2.0) + jar_wall / 2.0, 0.0, rim_z)),
        material=dark_rubber,
        name="left_rim",
    )
    jar.visual(
        Box((jar_wall, jar_inner, jar_rim_height)),
        origin=Origin(xyz=((jar_outer / 2.0) - jar_wall / 2.0, 0.0, rim_z)),
        material=dark_rubber,
        name="right_rim",
    )
    hinge_axis_y = (jar_outer / 2.0) + 0.006
    hinge_axis_z = jar_height + 0.008
    jar.inertial = Inertial.from_geometry(
        Box((jar_outer, jar_outer, jar_height)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, jar_height / 2.0)),
    )

    blades = model.part("blade_assembly")
    blades.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="hub",
    )
    blades.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=steel,
        name="bearing_flange",
    )
    blades.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=steel,
        name="spindle_cap",
    )
    blade_angles = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    blade_pitches = (0.22, -0.22, -0.22, 0.22)
    blade_z = (0.012, 0.012, 0.012, 0.012)
    for index, (angle, pitch, z) in enumerate(zip(blade_angles, blade_pitches, blade_z), start=1):
        blades.visual(
            Box((0.044, 0.010, 0.002)),
            origin=Origin(xyz=(0.019 * math.cos(angle), 0.019 * math.sin(angle), z), rpy=(0.0, pitch, angle)),
            material=steel,
            name=f"blade_{index}",
        )
    blades.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.022),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -0.088, -0.001)),
        material=dark_rubber,
        name="lid_shell",
    )
    lid.visual(
        Box((lid_plug_size, lid_plug_size, lid_plug_depth)),
        origin=Origin(xyz=(0.0, -hinge_axis_y, -0.014)),
        material=charcoal,
        name="lid_plug",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.152, -0.001)),
        material=accent,
        name="lid_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_thickness)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.088, -0.001)),
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, seat_top_z)),
    )
    model.articulation(
        "jar_to_blades",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blades,
        origin=Origin(xyz=(0.0, 0.0, jar_collar_height + 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=18.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blades = object_model.get_part("blade_assembly")
    lid = object_model.get_part("lid")

    blade_joint = object_model.get_articulation("jar_to_blades")
    lid_hinge = object_model.get_articulation("jar_to_lid")

    jar_seat = base.get_visual("jar_seat")
    body_shell = base.get_visual("body_shell")
    jar_collar = jar.get_visual("collar")
    front_rim = jar.get_visual("front_rim")
    rear_rim = jar.get_visual("rear_rim")
    bearing_flange = blades.get_visual("bearing_flange")
    hub = blades.get_visual("hub")
    lid_shell = lid.get_visual("lid_shell")
    lid_plug = lid.get_visual("lid_plug")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_overlap(jar, base, axes="xy", min_overlap=0.020, elem_a=jar_collar, elem_b=jar_seat)
    ctx.expect_within(jar, base, axes="xy")
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=jar_collar,
        negative_elem=jar_seat,
        name="jar_collar_seats_on_base",
    )
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        min_gap=0.280,
        positive_elem=rear_rim,
        negative_elem=body_shell,
        name="jar_towers_above_heavy_base",
    )
    ctx.expect_overlap(blades, jar, axes="xy", min_overlap=0.060)
    ctx.expect_within(blades, jar, axes="xy")
    ctx.expect_gap(
        blades,
        jar,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bearing_flange,
        negative_elem=jar_collar,
        name="blade_bearing_flange_seats_on_jar_collar",
    )
    ctx.expect_gap(
        blades,
        jar,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem=hub,
        negative_elem=jar_collar,
        name="blade_hub_rides_just_above_jar_collar",
    )
    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.100, elem_a=lid_plug, elem_b=jar_collar)
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=lid_shell,
        negative_elem=front_rim,
        name="lid_sits_on_front_rim",
    )
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=lid_shell,
        negative_elem=rear_rim,
        name="lid_sits_on_rear_rim",
    )
    ctx.expect_origin_distance(blades, jar, axes="xy", max_dist=0.001)

    with ctx.pose({blade_joint: math.pi / 2.0}):
        ctx.expect_within(blades, jar, axes="xy")
        ctx.expect_overlap(blades, jar, axes="xy", min_overlap=0.060)

    with ctx.pose({lid_hinge: 1.35}):
        ctx.expect_gap(
            lid,
            jar,
            axis="z",
            min_gap=0.015,
            positive_elem=lid_plug,
            negative_elem=rear_rim,
            name="open_lid_lifts_plug_clear_of_jar_mouth",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
