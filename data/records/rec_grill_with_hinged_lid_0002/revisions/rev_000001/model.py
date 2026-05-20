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
    DomeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BOWL_RADIUS = 0.235
LID_RADIUS = 0.242
LID_CENTER_Z = 0.004
HINGE_Y = -0.252
HINGE_Z = 0.040
LEG_RADIUS = 0.011
BRACE_RADIUS = 0.0045


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / name)


def _polar_xyz(radius: float, angle_deg: float, z: float) -> tuple[float, float, float]:
    angle = math.radians(angle_deg)
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _lerp_point(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_kettle_grill", assets=ASSETS)

    black_enamel = model.material("black_enamel", rgba=(0.10, 0.10, 0.11, 1.0))
    powder_steel = model.material("powder_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.38, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.68, 0.70, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.56, 0.34, 0.18, 1.0))

    def lid_local(x: float, y: float, z: float) -> tuple[float, float, float]:
        return (x, y - HINGE_Y, z - HINGE_Z)

    base = model.part("base")

    bowl_geom = DomeGeometry(
        radius=BOWL_RADIUS,
        radial_segments=48,
        height_segments=20,
        closed=True,
    ).rotate_x(math.pi)
    base.visual(
        _mesh("kettle_bowl_shell.obj", bowl_geom),
        material=black_enamel,
        name="bowl_shell",
    )

    collar_geom = LatheGeometry(
        [
            (BOWL_RADIUS - 0.004, -0.040),
            (BOWL_RADIUS + 0.022, -0.040),
            (BOWL_RADIUS + 0.022, -0.010),
            (BOWL_RADIUS - 0.004, -0.010),
        ],
        segments=56,
    )
    base.visual(
        _mesh("kettle_collar_band.obj", collar_geom),
        material=powder_steel,
        name="collar_band",
    )

    left_top = _polar_xyz(BOWL_RADIUS + 0.020, 150.0, -0.018)
    right_top = _polar_xyz(BOWL_RADIUS + 0.020, 30.0, -0.018)
    rear_top = _polar_xyz(BOWL_RADIUS + 0.020, -90.0, -0.018)

    left_bottom = _polar_xyz(0.345, 150.0, -0.602)
    right_bottom = _polar_xyz(0.345, 30.0, -0.602)
    rear_bottom = _polar_xyz(0.345, -90.0, -0.602)

    base.visual(
        _mesh(
            "kettle_left_leg.obj",
            wire_from_points([left_top, left_bottom], radius=LEG_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=powder_steel,
        name="left_leg",
    )
    base.visual(
        _mesh(
            "kettle_right_leg.obj",
            wire_from_points([right_top, right_bottom], radius=LEG_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=powder_steel,
        name="right_leg",
    )
    base.visual(
        _mesh(
            "kettle_rear_leg.obj",
            wire_from_points([rear_top, rear_bottom], radius=LEG_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=powder_steel,
        name="rear_leg",
    )

    left_brace = _lerp_point(left_top, left_bottom, 0.70)
    right_brace = _lerp_point(right_top, right_bottom, 0.70)
    rear_brace = _lerp_point(rear_top, rear_bottom, 0.70)

    base.visual(
        _mesh(
            "kettle_front_brace.obj",
            wire_from_points([left_brace, right_brace], radius=BRACE_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=dark_steel,
        name="front_brace",
    )
    base.visual(
        _mesh(
            "kettle_right_brace.obj",
            wire_from_points([right_brace, rear_brace], radius=BRACE_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=dark_steel,
        name="right_brace",
    )
    base.visual(
        _mesh(
            "kettle_left_brace.obj",
            wire_from_points([rear_brace, left_brace], radius=BRACE_RADIUS, cap_ends=True, corner_mode="miter"),
        ),
        material=dark_steel,
        name="left_brace",
    )

    base.visual(Sphere(radius=0.015), origin=Origin(xyz=left_bottom), material=dark_steel, name="left_foot")
    base.visual(Sphere(radius=0.015), origin=Origin(xyz=right_bottom), material=dark_steel, name="right_foot")
    base.visual(Sphere(radius=0.015), origin=Origin(xyz=rear_bottom), material=dark_steel, name="rear_foot")

    base.visual(
        Cylinder(radius=0.056, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -BOWL_RADIUS - 0.003)),
        material=brushed_steel,
        name="bottom_vent_plate",
    )
    base.visual(
        Box((0.040, 0.012, 0.006)),
        origin=Origin(xyz=(0.048, 0.0, -BOWL_RADIUS - 0.003)),
        material=brushed_steel,
        name="bottom_vent_tab",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.269)),
        material=powder_steel,
        name="ash_pedestal",
    )
    base.visual(
        Cylinder(radius=0.100, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.308)),
        material=powder_steel,
        name="ash_tray",
    )

    base.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(-0.094, -0.264, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_steel,
        name="left_hinge_knuckle",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.094, -0.264, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_steel,
        name="right_hinge_knuckle",
    )
    base.visual(
        Box((0.020, 0.012, 0.072)),
        origin=Origin(xyz=(-0.094, -0.272, -0.002)),
        material=powder_steel,
        name="left_hinge_bracket",
    )
    base.visual(
        Box((0.020, 0.012, 0.072)),
        origin=Origin(xyz=(0.094, -0.272, -0.002)),
        material=powder_steel,
        name="right_hinge_bracket",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(0.0, -0.266, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_steel,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.030, 0.040, 0.024)),
        origin=Origin(xyz=(-0.094, -0.257, -0.014)),
        material=powder_steel,
        name="left_hinge_gusset",
    )
    base.visual(
        Box((0.030, 0.040, 0.024)),
        origin=Origin(xyz=(0.094, -0.257, -0.014)),
        material=powder_steel,
        name="right_hinge_gusset",
    )
    base.visual(
        Box((0.228, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, -0.255, -0.025)),
        material=powder_steel,
        name="rear_hinge_mount",
    )

    rack_hinge_origin = (-0.282, 0.162, -0.020)
    base.visual(
        Box((0.075, 0.020, 0.020)),
        origin=Origin(xyz=(-0.250, 0.145, -0.020)),
        material=powder_steel,
        name="rack_hinge_mount",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=rack_hinge_origin),
        material=powder_steel,
        name="rack_hinge_boss",
    )

    lid = model.part("lid")

    lid.visual(
        _mesh("kettle_lid_dome.obj", DomeGeometry(radius=LID_RADIUS, radial_segments=48, height_segments=20, closed=True)),
        origin=Origin(xyz=lid_local(0.0, 0.0, LID_CENTER_Z)),
        material=black_enamel,
        name="lid_shell",
    )

    lid_skirt_geom = LatheGeometry(
        [
            (LID_RADIUS - 0.006, -0.004),
            (LID_RADIUS + 0.004, -0.004),
            (LID_RADIUS + 0.004, 0.018),
            (LID_RADIUS - 0.020, 0.018),
        ],
        segments=56,
    )
    lid.visual(
        _mesh("kettle_lid_skirt.obj", lid_skirt_geom),
        origin=Origin(xyz=lid_local(0.0, 0.0, LID_CENTER_Z)),
        material=black_enamel,
        name="lid_skirt",
    )

    lid.visual(
        Cylinder(radius=0.007, length=0.180),
        origin=Origin(xyz=lid_local(0.0, -0.226, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_steel,
        name="rear_hinge_leaf",
    )

    lid.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=lid_local(-0.086, 0.080, 0.239)),
        material=powder_steel,
        name="left_handle_post",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=lid_local(0.086, 0.080, 0.239)),
        material=powder_steel,
        name="right_handle_post",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.188),
        origin=Origin(xyz=lid_local(0.0, 0.080, 0.266), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_wood,
        name="wood_handle",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=lid_local(0.0, 0.0, LID_CENTER_Z + LID_RADIUS + 0.003)),
        material=brushed_steel,
        name="vent_pedestal",
    )

    vent = model.part("top_vent")
    vent.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="vent_cap",
    )
    vent.visual(
        Box((0.022, 0.008, 0.006)),
        origin=Origin(xyz=(0.030, 0.0, 0.013)),
        material=brushed_steel,
        name="vent_tab",
    )

    rack = model.part("warming_rack")
    rack.visual(
        Cylinder(radius=0.015, length=0.034),
        material=powder_steel,
        name="rack_sleeve",
    )
    rack_loop_geom = wire_from_points(
        [
            (-0.030, 0.050, 0.102),
            (-0.170, 0.050, 0.102),
            (-0.170, 0.180, 0.102),
            (-0.030, 0.180, 0.102),
        ],
        radius=0.005,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=8,
    )
    rack.visual(_mesh("warming_rack_loop.obj", rack_loop_geom), material=powder_steel, name="rack_loop")
    rack.visual(
        _mesh(
            "warming_rack_support_a.obj",
            wire_from_points([(0.0, 0.0, 0.014), (-0.030, 0.060, 0.102)], radius=0.005, cap_ends=True, corner_mode="miter"),
        ),
        material=powder_steel,
        name="rack_support_a",
    )
    rack.visual(
        _mesh(
            "warming_rack_support_b.obj",
            wire_from_points([(0.0, 0.0, 0.014), (-0.030, 0.170, 0.102)], radius=0.005, cap_ends=True, corner_mode="miter"),
        ),
        material=powder_steel,
        name="rack_support_b",
    )
    rack.visual(
        _mesh(
            "warming_rack_cross_a.obj",
            wire_from_points([(-0.078, 0.050, 0.102), (-0.078, 0.180, 0.102)], radius=0.0036, cap_ends=True, corner_mode="miter"),
        ),
        material=dark_steel,
        name="rack_cross_a",
    )
    rack.visual(
        _mesh(
            "warming_rack_cross_b.obj",
            wire_from_points([(-0.122, 0.050, 0.102), (-0.122, 0.180, 0.102)], radius=0.0036, cap_ends=True, corner_mode="miter"),
        ),
        material=dark_steel,
        name="rack_cross_b",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.32),
    )
    model.articulation(
        "vent_spin",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=vent,
        origin=Origin(xyz=lid_local(0.0, 0.0, LID_CENTER_Z + LID_RADIUS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "rack_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rack,
        origin=Origin(xyz=rack_hinge_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    vent = object_model.get_part("top_vent")
    rack = object_model.get_part("warming_rack")

    lid_hinge = object_model.get_articulation("lid_hinge")
    vent_spin = object_model.get_articulation("vent_spin")
    rack_swing = object_model.get_articulation("rack_swing")

    bowl_shell = base.get_visual("bowl_shell")
    collar_band = base.get_visual("collar_band")
    bottom_vent_plate = base.get_visual("bottom_vent_plate")
    ash_tray = base.get_visual("ash_tray")
    left_hinge_knuckle = base.get_visual("left_hinge_knuckle")
    right_hinge_knuckle = base.get_visual("right_hinge_knuckle")
    front_brace = base.get_visual("front_brace")
    left_foot = base.get_visual("left_foot")
    right_foot = base.get_visual("right_foot")
    rear_foot = base.get_visual("rear_foot")
    rack_hinge_boss = base.get_visual("rack_hinge_boss")

    lid_shell = lid.get_visual("lid_shell")
    lid_skirt = lid.get_visual("lid_skirt")
    wood_handle = lid.get_visual("wood_handle")
    vent_pedestal = lid.get_visual("vent_pedestal")

    vent_cap = vent.get_visual("vent_cap")

    rack_sleeve = rack.get_visual("rack_sleeve")
    rack_loop = rack.get_visual("rack_loop")

    ctx.allow_overlap(vent, lid, reason="rotating chimney cap sleeves over the fixed crown pedestal")
    ctx.allow_overlap(rack, base, reason="warming rack sleeve nests around the left collar hinge boss")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.34)
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=-0.001,
        max_gap=0.001,
        positive_elem=lid_skirt,
        negative_elem=bowl_shell,
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="z",
        min_gap=0.003,
        positive_elem=wood_handle,
        negative_elem=lid_shell,
    )
    ctx.expect_gap(
        base,
        base,
        axis="z",
        min_gap=0.055,
        positive_elem=bottom_vent_plate,
        negative_elem=ash_tray,
    )
    ctx.expect_gap(
        base,
        base,
        axis="z",
        min_gap=0.012,
        positive_elem=left_hinge_knuckle,
        negative_elem=bowl_shell,
    )
    ctx.expect_gap(
        base,
        base,
        axis="z",
        min_gap=0.012,
        positive_elem=right_hinge_knuckle,
        negative_elem=bowl_shell,
    )
    ctx.expect_gap(
        base,
        base,
        axis="z",
        min_gap=0.200,
        positive_elem=collar_band,
        negative_elem=front_brace,
    )
    ctx.expect_gap(
        base,
        base,
        axis="x",
        min_gap=0.048,
        positive_elem=bowl_shell,
        negative_elem=left_foot,
    )
    ctx.expect_gap(
        base,
        base,
        axis="x",
        min_gap=0.048,
        positive_elem=right_foot,
        negative_elem=bowl_shell,
    )
    ctx.expect_gap(
        base,
        base,
        axis="y",
        min_gap=0.070,
        positive_elem=bowl_shell,
        negative_elem=rear_foot,
    )
    ctx.expect_contact(vent, lid, elem_a=vent_cap, elem_b=vent_pedestal)
    ctx.expect_contact(rack, base, elem_a=rack_sleeve, elem_b=rack_hinge_boss)
    ctx.expect_gap(
        rack,
        base,
        axis="z",
        min_gap=0.080,
        positive_elem=rack_loop,
        negative_elem=collar_band,
    )

    with ctx.pose({vent_spin: 2.0}):
        ctx.expect_contact(vent, lid, elem_a=vent_cap, elem_b=vent_pedestal)

    with ctx.pose({lid_hinge: 1.25}):
        ctx.expect_gap(
            vent,
            base,
            axis="z",
            min_gap=0.060,
            positive_elem=vent_cap,
            negative_elem=bowl_shell,
        )

    with ctx.pose({rack_swing: 1.0}):
        ctx.expect_gap(
            base,
            rack,
            axis="x",
            min_gap=0.060,
            positive_elem=bowl_shell,
            negative_elem=rack_loop,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
