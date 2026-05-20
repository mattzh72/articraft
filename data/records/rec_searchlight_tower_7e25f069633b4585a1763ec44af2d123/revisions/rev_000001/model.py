from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _rod_between(part, start, end, radius: float, material, name: str) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_searchlight_tower")

    safety_yellow = _mat(model, "powder_coated_safety_yellow", (0.96, 0.68, 0.12, 1.0))
    dark_steel = _mat(model, "dark_parkerized_steel", (0.08, 0.085, 0.08, 1.0))
    black_rubber = _mat(model, "black_rubber", (0.015, 0.014, 0.013, 1.0))
    galvanized = _mat(model, "galvanized_wear_plate", (0.55, 0.57, 0.55, 1.0))
    lens_glass = _mat(model, "pale_blue_tempered_glass", (0.50, 0.78, 0.95, 0.62))
    reflector = _mat(model, "brushed_reflector", (0.86, 0.83, 0.72, 1.0))
    warning_red = _mat(model, "red_service_marking", (0.82, 0.05, 0.035, 1.0))

    tower = model.part("tower")

    # Heavy skid base and outriggers: all members overlap slightly at weld seams.
    tower.visual(Box((1.15, 0.72, 0.09)), origin=Origin(xyz=(0.0, 0.0, 0.045)), material=dark_steel, name="skid_frame")
    tower.visual(Box((1.45, 0.13, 0.08)), origin=Origin(xyz=(0.0, 0.36, 0.06)), material=dark_steel, name="front_outrigger")
    tower.visual(Box((1.45, 0.13, 0.08)), origin=Origin(xyz=(0.0, -0.36, 0.06)), material=dark_steel, name="rear_outrigger")
    tower.visual(Box((0.16, 0.96, 0.07)), origin=Origin(xyz=(0.58, 0.0, 0.055)), material=dark_steel, name="side_outrigger_0")
    tower.visual(Box((0.16, 0.96, 0.07)), origin=Origin(xyz=(-0.58, 0.0, 0.055)), material=dark_steel, name="side_outrigger_1")
    for x in (-0.62, 0.62):
        for y in (-0.43, 0.43):
            tower.visual(Box((0.22, 0.18, 0.035)), origin=Origin(xyz=(x, y, 0.018)), material=black_rubber, name=f"leveling_pad_{x}_{y}")
            tower.visual(
                Cylinder(radius=0.035, length=0.06),
                origin=Origin(xyz=(x, y, 0.095)),
                material=galvanized,
                name=f"leveling_screw_{x}_{y}",
            )

    # Serviceable ballast/control cabinet bolted to the skid.
    tower.visual(Box((0.56, 0.38, 0.34)), origin=Origin(xyz=(-0.28, -0.03, 0.25)), material=safety_yellow, name="service_cabinet")
    tower.visual(Box((0.50, 0.018, 0.23)), origin=Origin(xyz=(-0.28, -0.231, 0.27)), material=galvanized, name="access_door")
    tower.visual(Box((0.20, 0.012, 0.025)), origin=Origin(xyz=(-0.28, -0.238, 0.30)), material=dark_steel, name="door_pull")
    tower.visual(Box((0.09, 0.020, 0.19)), origin=Origin(xyz=(-0.56, -0.232, 0.27)), material=dark_steel, name="door_hinge_strip")
    tower.visual(Box((0.035, 0.026, 0.055)), origin=Origin(xyz=(-0.04, -0.238, 0.27)), material=warning_red, name="lockout_tag")

    # Mast socket, collars, and triangulated tower bracing.
    tower.visual(Cylinder(radius=0.115, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.15)), material=dark_steel, name="mast_socket")
    tower.visual(Cylinder(radius=0.064, length=1.25), origin=Origin(xyz=(0.0, 0.0, 0.73)), material=safety_yellow, name="main_mast")
    tower.visual(Cylinder(radius=0.086, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.43)), material=galvanized, name="lower_mast_collar")
    tower.visual(Cylinder(radius=0.082, length=0.075), origin=Origin(xyz=(0.0, 0.0, 1.05)), material=galvanized, name="upper_mast_collar")
    tower.visual(Cylinder(radius=0.105, length=0.070), origin=Origin(xyz=(0.0, 0.0, 1.39)), material=dark_steel, name="bearing_plinth")
    tower.visual(Cylinder(radius=0.145, length=0.045), origin=Origin(xyz=(0.0, 0.0, 1.44)), material=galvanized, name="slew_lower_race")
    for i, (sx, sy) in enumerate(((0.45, 0.30), (-0.45, 0.30), (0.45, -0.30), (-0.45, -0.30))):
        _rod_between(tower, (sx, sy, 0.11), (0.0, 0.0, 0.66), 0.018, dark_steel, f"tower_brace_{i}")
        _rod_between(tower, (sx * 0.72, sy * 0.72, 0.42), (0.0, 0.0, 1.18), 0.014, dark_steel, f"upper_brace_{i}")

    # Captive clamp handles and replaceable wear pads on the collars.
    clamp_knob_mesh = mesh_from_geometry(
        KnobGeometry(0.075, 0.040, body_style="lobed", grip=KnobGrip(style="ribbed", count=8, depth=0.004)),
        "field_lobed_clamp_knob",
    )
    for z, suffix in ((0.43, "lower"), (1.05, "upper")):
        tower.visual(Cylinder(radius=0.018, length=0.18), origin=Origin(xyz=(0.09, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name=f"{suffix}_clamp_screw")
        tower.visual(clamp_knob_mesh, origin=Origin(xyz=(0.185, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)), material=black_rubber, name=f"{suffix}_clamp_knob")
        tower.visual(Box((0.035, 0.11, 0.055)), origin=Origin(xyz=(-0.078, 0.0, z)), material=galvanized, name=f"{suffix}_wear_pad")

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(Cylinder(radius=0.142, length=0.04), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=dark_steel, name="slew_turntable")
    pan_yoke.visual(Cylinder(radius=0.082, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=galvanized, name="pan_bearing_hub")
    pan_yoke.visual(Box((0.28, 0.20, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.15)), material=dark_steel, name="yoke_pedestal")
    pan_yoke.visual(Box((0.36, 0.11, 0.07)), origin=Origin(xyz=(0.0, -0.105, 0.205)), material=dark_steel, name="rear_stiffener")
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.56, 0.24, 0.42),
            span_width=0.39,
            trunnion_diameter=0.082,
            trunnion_center_z=0.285,
            base_thickness=0.07,
            corner_radius=0.012,
            center=False,
        ),
        "wide_service_trunnion_yoke",
    )
    pan_yoke.visual(yoke_mesh, origin=Origin(xyz=(0.0, 0.0, 0.19)), material=safety_yellow, name="trunnion_yoke")
    for x, name in ((-0.286, "bearing_pad_0"), (0.286, "bearing_pad_1")):
        pan_yoke.visual(Box((0.018, 0.11, 0.045)), origin=Origin(xyz=(x, 0.064, 0.530)), material=galvanized, name=name)
    pan_yoke.visual(Box((0.14, 0.09, 0.08)), origin=Origin(xyz=(0.0, 0.135, 0.245)), material=dark_steel, name="tilt_stop_block")
    pan_yoke.visual(Box((0.050, 0.15, 0.035)), origin=Origin(xyz=(0.255, 0.0, 0.56)), material=black_rubber, name="upper_bumper")
    pan_yoke.visual(Box((0.050, 0.15, 0.035)), origin=Origin(xyz=(0.255, 0.0, 0.36)), material=black_rubber, name="lower_bumper")

    spotlight = model.part("spotlight")
    # Child frame is the tilt axis through the trunnions. The barrel points along +Y at q=0.
    spotlight.visual(Cylinder(radius=0.165, length=0.46), origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_steel, name="lamp_housing")
    spotlight.visual(Cylinder(radius=0.183, length=0.045), origin=Origin(xyz=(0.0, 0.285, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=safety_yellow, name="front_bezel")
    spotlight.visual(Cylinder(radius=0.139, length=0.018), origin=Origin(xyz=(0.0, 0.316, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=lens_glass, name="front_lens")
    spotlight.visual(Cylinder(radius=0.105, length=0.012), origin=Origin(xyz=(0.0, 0.304, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=reflector, name="reflector_disk")
    spotlight.visual(Cylinder(radius=0.146, length=0.035), origin=Origin(xyz=(0.0, -0.210, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=galvanized, name="rear_service_cover")
    spotlight.visual(Box((0.16, 0.025, 0.05)), origin=Origin(xyz=(0.0, -0.232, 0.08)), material=dark_steel, name="rear_cover_handle")
    spotlight.visual(Cylinder(radius=0.035, length=0.59), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="tilt_shaft")
    for x, name in ((-0.170, "trunnion_boss_0"), (0.170, "trunnion_boss_1")):
        spotlight.visual(Cylinder(radius=0.065, length=0.04), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=safety_yellow, name=name)
    # Cooling fins are welded to the lamp housing; slight embed prevents floating slivers.
    for i, z in enumerate((0.082, 0.115, 0.148)):
        spotlight.visual(Box((0.23, 0.24, 0.020)), origin=Origin(xyz=(0.0, -0.015, z)), material=galvanized, name=f"top_cooling_fin_{i}")
    spotlight.visual(Box((0.10, 0.030, 0.05)), origin=Origin(xyz=(0.0, 0.205, -0.135)), material=warning_red, name="aiming_index")
    spotlight.visual(Box((0.24, 0.055, 0.045)), origin=Origin(xyz=(0.0, -0.02, -0.165)), material=black_rubber, name="cable_gland")
    _rod_between(spotlight, (-0.09, -0.18, -0.15), (-0.09, -0.30, -0.26), 0.014, black_rubber, "strain_relief_0")
    _rod_between(spotlight, (0.09, -0.18, -0.15), (0.09, -0.30, -0.26), 0.014, black_rubber, "strain_relief_1")
    spotlight.visual(Box((0.23, 0.06, 0.035)), origin=Origin(xyz=(0.0, -0.31, -0.275)), material=black_rubber, name="cable_clamp")

    model.articulation(
        "tower_to_pan_yoke",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.4625)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8),
    )
    model.articulation(
        "pan_yoke_to_spotlight",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight = object_model.get_part("spotlight")
    pan_joint = object_model.get_articulation("tower_to_pan_yoke")
    tilt_joint = object_model.get_articulation("pan_yoke_to_spotlight")

    ctx.expect_gap(
        pan_yoke,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slew_turntable",
        negative_elem="slew_lower_race",
        name="slew turntable sits on lower race",
    )
    ctx.expect_within(
        spotlight,
        pan_yoke,
        axes="x",
        inner_elem="lamp_housing",
        outer_elem="trunnion_yoke",
        margin=0.01,
        name="lamp housing fits between yoke cheeks",
    )
    ctx.expect_overlap(
        spotlight,
        pan_yoke,
        axes="x",
        elem_a="tilt_shaft",
        elem_b="trunnion_yoke",
        min_overlap=0.45,
        name="tilt shaft spans both bearing blocks",
    )

    closed_lens = ctx.part_element_world_aabb(spotlight, elem="front_lens")
    with ctx.pose({tilt_joint: 0.55}):
        tilted_lens = ctx.part_element_world_aabb(spotlight, elem="front_lens")
    ctx.check(
        "tilt raises front lens",
        closed_lens is not None
        and tilted_lens is not None
        and ((tilted_lens[0][2] + tilted_lens[1][2]) * 0.5) > ((closed_lens[0][2] + closed_lens[1][2]) * 0.5) + 0.09,
        details=f"closed={closed_lens}, tilted={tilted_lens}",
    )

    rest_lens = ctx.part_element_world_aabb(spotlight, elem="front_lens")
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = ctx.part_element_world_aabb(spotlight, elem="front_lens")
    ctx.check(
        "pan swings head around mast",
        rest_lens is not None
        and panned_lens is not None
        and abs(((panned_lens[0][0] + panned_lens[1][0]) * 0.5) - ((rest_lens[0][0] + rest_lens[1][0]) * 0.5)) > 0.20,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
