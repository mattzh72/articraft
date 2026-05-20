from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 20,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_cannon")

    blackened_iron = model.material("blackened_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.29, 0.31, 1.0))
    oily_steel = model.material("oily_steel", rgba=(0.42, 0.43, 0.45, 1.0))
    soot = model.material("soot", rgba=(0.05, 0.05, 0.05, 1.0))

    arm_outer = _translate_profile(
        rounded_rect_profile(0.20, 0.30, radius=0.018, corner_segments=6),
        0.15,
        0.29,
    )
    arm_hole = _circle_profile(0.024, center=(0.18, 0.305), segments=24)
    arm_geom = ExtrudeWithHolesGeometry(
        arm_outer,
        [arm_hole],
        0.024,
        cap=True,
        center=True,
    )
    arm_geom.rotate_x(math.pi / 2.0)
    yoke_arm_mesh = mesh_from_geometry(arm_geom, "yoke_arm")

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.120, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=blackened_iron,
        name="deck_flange",
    )
    mount.visual(
        Cylinder(radius=0.072, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=gunmetal,
        name="base_coller",
    )
    mount.visual(
        Cylinder(radius=0.055, length=0.750),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=blackened_iron,
        name="main_post",
    )
    mount.visual(
        Cylinder(radius=0.075, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=gunmetal,
        name="shoulder_collar",
    )
    mount.visual(
        Cylinder(radius=0.064, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        material=oily_steel,
        name="pintle_post",
    )
    mount.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.014)),
        material=oily_steel,
        name="pintle_cap",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=1.04),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.172, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, 0.077, 0.080)),
        material=gunmetal,
        name="pivot_sleeve_front",
    )
    yoke.visual(
        Box((0.172, 0.018, 0.160)),
        origin=Origin(xyz=(0.0, -0.077, 0.080)),
        material=gunmetal,
        name="pivot_sleeve_back",
    )
    yoke.visual(
        Box((0.018, 0.136, 0.160)),
        origin=Origin(xyz=(0.077, 0.0, 0.080)),
        material=gunmetal,
        name="pivot_sleeve_right",
    )
    yoke.visual(
        Box((0.018, 0.136, 0.160)),
        origin=Origin(xyz=(-0.077, 0.0, 0.080)),
        material=gunmetal,
        name="pivot_sleeve_left",
    )
    yoke.visual(
        Box((0.094, 0.046, 0.074)),
        origin=Origin(xyz=(0.047, 0.104, 0.180)),
        material=gunmetal,
        name="right_sleeve_lug",
    )
    yoke.visual(
        Box((0.094, 0.046, 0.074)),
        origin=Origin(xyz=(0.047, -0.104, 0.180)),
        material=gunmetal,
        name="left_sleeve_lug",
    )
    yoke.visual(
        Box((0.160, 0.060, 0.055)),
        origin=Origin(xyz=(0.110, 0.112, 0.170)),
        material=gunmetal,
        name="right_lower_cheek",
    )
    yoke.visual(
        Box((0.160, 0.060, 0.055)),
        origin=Origin(xyz=(0.110, -0.112, 0.170)),
        material=gunmetal,
        name="left_lower_cheek",
    )
    yoke.visual(
        Box((0.095, 0.055, 0.085)),
        origin=Origin(xyz=(0.035, 0.112, 0.260)),
        material=gunmetal,
        name="right_upper_cheek",
    )
    yoke.visual(
        Box((0.095, 0.055, 0.085)),
        origin=Origin(xyz=(0.035, -0.112, 0.260)),
        material=gunmetal,
        name="left_upper_cheek",
    )
    yoke.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(0.0, 0.136, 0.0)),
        material=gunmetal,
        name="right_arm",
    )
    yoke.visual(
        yoke_arm_mesh,
        origin=Origin(xyz=(0.0, -0.136, 0.0)),
        material=gunmetal,
        name="left_arm",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.180, 0.148, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oily_steel,
        name="right_boss",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.180, -0.148, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oily_steel,
        name="left_boss",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.22, 0.32, 0.44)),
        mass=19.0,
        origin=Origin(xyz=(0.05, 0.0, 0.22)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.089, length=0.120),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="breech_ring",
    )
    barrel.visual(
        Cylinder(radius=0.111, length=0.090),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="first_reinforce_ring",
    )
    barrel.visual(
        Cylinder(radius=0.103, length=0.230),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="first_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.083, length=0.500),
        origin=Origin(xyz=(0.455, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="chase",
    )
    barrel.visual(
        Cylinder(radius=0.095, length=0.150),
        origin=Origin(xyz=(0.780, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="muzzle_swell",
    )
    barrel.visual(
        Cylinder(radius=0.108, length=0.050),
        origin=Origin(xyz=(0.880, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_rim",
    )
    barrel.visual(
        Cylinder(radius=0.032, length=0.090),
        origin=Origin(xyz=(0.855, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soot,
        name="bore_shadow",
    )
    barrel.visual(
        Cylinder(radius=0.018, length=0.248),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oily_steel,
        name="trunnion_pins",
    )
    barrel.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.037),
        origin=Origin(xyz=(-0.175, 0.0, 0.0)),
        material=gunmetal,
        name="cascabel_knob",
    )
    barrel.visual(
        Box((0.090, 0.030, 0.004)),
        origin=Origin(xyz=(0.040, 0.0, 0.109)),
        material=gunmetal,
        name="touchhole_seat",
    )
    barrel.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.048, 0.0, 0.107), rpy=(0.0, 0.0, 0.0)),
        material=soot,
        name="touchhole",
    )
    barrel.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.004, 0.013, 0.117), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hinge_lug_right",
    )
    barrel.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.004, -0.013, 0.117), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hinge_lug_left",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=1.05),
        mass=28.0,
        origin=Origin(xyz=(0.360, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    touchhole_cover = model.part("touchhole_cover")
    touchhole_cover.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oily_steel,
        name="cover_knuckle",
    )
    touchhole_cover.visual(
        Box((0.085, 0.028, 0.003)),
        origin=Origin(xyz=(0.0425, 0.0, -0.0045)),
        material=gunmetal,
        name="cover_plate",
    )
    touchhole_cover.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(0.068, 0.0, -0.0005)),
        material=gunmetal,
        name="cover_lift_tab",
    )
    touchhole_cover.inertial = Inertial.from_geometry(
        Box((0.085, 0.028, 0.012)),
        mass=0.25,
        origin=Origin(xyz=(0.0425, 0.0, 0.001)),
    )

    model.articulation(
        "post_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.8),
    )
    model.articulation(
        "yoke_to_barrel",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=barrel,
        origin=Origin(xyz=(0.180, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.75,
        ),
    )
    model.articulation(
        "barrel_to_touchhole_cover",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=touchhole_cover,
        origin=Origin(xyz=(-0.004, 0.0, 0.117)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount = object_model.get_part("mount")
    yoke = object_model.get_part("yoke")
    barrel = object_model.get_part("barrel")
    cover = object_model.get_part("touchhole_cover")

    post_to_yoke = object_model.get_articulation("post_to_yoke")
    yoke_to_barrel = object_model.get_articulation("yoke_to_barrel")
    barrel_to_cover = object_model.get_articulation("barrel_to_touchhole_cover")

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        yoke,
        mount,
        axis="y",
        positive_elem="pivot_sleeve_front",
        negative_elem="pintle_post",
        min_gap=0.003,
        max_gap=0.006,
        name="front sleeve strap clears the pintle with a small running gap",
    )
    ctx.expect_gap(
        yoke,
        mount,
        axis="x",
        positive_elem="pivot_sleeve_right",
        negative_elem="pintle_post",
        min_gap=0.003,
        max_gap=0.006,
        name="side sleeve strap clears the pintle with a small running gap",
    )
    ctx.expect_gap(
        yoke,
        mount,
        axis="z",
        positive_elem="pivot_sleeve_front",
        negative_elem="shoulder_collar",
        max_gap=0.004,
        max_penetration=0.0005,
        name="yoke sleeve bears on the post shoulder",
    )
    ctx.expect_gap(
        cover,
        barrel,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="touchhole_seat",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="touchhole cover rests on its breech seat when closed",
    )

    muzzle_rest = elem_center("barrel", "muzzle_rim")
    with ctx.pose({post_to_yoke: math.pi / 2.0}):
        muzzle_traversed = elem_center("barrel", "muzzle_rim")
    ctx.check(
        "barrel traverses horizontally about the post axis",
        muzzle_rest is not None
        and muzzle_traversed is not None
        and muzzle_rest[0] > 0.85
        and abs(muzzle_rest[1]) < 0.05
        and muzzle_traversed[1] > 0.85
        and abs(muzzle_traversed[0]) < 0.05,
        details=f"rest={muzzle_rest}, traversed={muzzle_traversed}",
    )

    muzzle_level = elem_center("barrel", "muzzle_rim")
    with ctx.pose({yoke_to_barrel: 0.65}):
        muzzle_elevated = elem_center("barrel", "muzzle_rim")
    ctx.check(
        "barrel elevates upward at the trunnions",
        muzzle_level is not None
        and muzzle_elevated is not None
        and muzzle_elevated[2] > muzzle_level[2] + 0.25,
        details=f"level={muzzle_level}, elevated={muzzle_elevated}",
    )

    cover_closed = elem_center("touchhole_cover", "cover_plate")
    with ctx.pose({barrel_to_cover: 1.20}):
        cover_open = elem_center("touchhole_cover", "cover_plate")
    ctx.check(
        "touchhole cover flips up from the breech",
        cover_closed is not None
        and cover_open is not None
        and cover_open[2] > cover_closed[2] + 0.030,
        details=f"closed={cover_closed}, open={cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
