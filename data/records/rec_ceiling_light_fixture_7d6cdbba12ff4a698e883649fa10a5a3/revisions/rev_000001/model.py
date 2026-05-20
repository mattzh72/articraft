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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_arm_ceiling_spot_fixture")

    painted_metal = model.material("painted_metal", rgba=(0.93, 0.93, 0.91, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Cylinder(radius=0.10, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=painted_metal,
        name="plate_disc",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_metal,
        name="plate_socket",
    )

    rotating_bar = model.part("rotating_bar")
    rotating_bar.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_metal,
        name="swivel_stem",
    )
    rotating_bar.visual(
        Cylinder(radius=0.009, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=painted_metal,
        name="center_rod",
    )
    rotating_bar.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=satin_metal,
        name="branch_hub",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        rotating_bar.visual(
            Cylinder(radius=0.008, length=0.160),
            origin=Origin(
                xyz=(side_sign * 0.080, 0.0, 0.172),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=painted_metal,
            name=f"{side_name}_arm",
        )
        rotating_bar.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(
                xyz=(side_sign * 0.165, 0.0, 0.172),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_metal,
            name=f"{side_name}_hinge_barrel",
        )
    rotating_bar.visual(
        Box((0.016, 0.006, 0.030)),
        origin=Origin(xyz=(-0.173, -0.012, 0.172)),
        material=satin_metal,
        name="left_inner_yoke",
    )
    rotating_bar.visual(
        Box((0.016, 0.006, 0.030)),
        origin=Origin(xyz=(-0.173, 0.012, 0.172)),
        material=satin_metal,
        name="left_outer_yoke",
    )
    rotating_bar.visual(
        Box((0.016, 0.006, 0.030)),
        origin=Origin(xyz=(0.173, -0.012, 0.172)),
        material=satin_metal,
        name="right_inner_yoke",
    )
    rotating_bar.visual(
        Box((0.016, 0.006, 0.030)),
        origin=Origin(xyz=(0.173, 0.012, 0.172)),
        material=satin_metal,
        name="right_outer_yoke",
    )

    rod_limits = MotionLimits(
        effort=10.0,
        velocity=1.5,
        lower=-math.radians(165.0),
        upper=math.radians(165.0),
    )
    model.articulation(
        "plate_to_bar",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=rotating_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=rod_limits,
    )

    spot_shell_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.010, 0.000),
            (0.019, 0.012),
            (0.026, 0.034),
            (0.028, 0.082),
        ],
        inner_profile=[
            (0.000, 0.006),
            (0.017, 0.016),
            (0.023, 0.034),
            (0.022, 0.078),
        ],
        segments=40,
    )
    spot_shell_mesh = mesh_from_geometry(spot_shell_geom, "spot_shell")

    def add_spot_head(part_name: str, side_sign: float) -> None:
        head = model.part(part_name)
        shell_yaw = math.pi / 2.0 if side_sign > 0.0 else -math.pi / 2.0
        head.visual(
            spot_shell_mesh,
            origin=Origin(
                xyz=(side_sign * 0.018, 0.0, 0.0),
                rpy=(0.0, shell_yaw, 0.0),
            ),
            material=painted_metal,
            name="spot_shell",
        )
        head.visual(
            Cylinder(radius=0.021, length=0.070),
            origin=Origin(
                xyz=(side_sign * 0.047, 0.0, 0.0),
                rpy=(0.0, shell_yaw, 0.0),
            ),
            material=dark_trim,
            name="lamp_core",
        )
        head.visual(
            Cylinder(radius=0.0285, length=0.006),
            origin=Origin(
                xyz=(side_sign * 0.079, 0.0, 0.0),
                rpy=(0.0, shell_yaw, 0.0),
            ),
            material=satin_metal,
            name="front_bezel",
        )
        head.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(
                xyz=(side_sign * 0.008, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_metal,
            name="hinge_trunnion",
        )

    add_spot_head("left_head", -1.0)
    add_spot_head("right_head", 1.0)

    tilt_limits = MotionLimits(
        effort=4.0,
        velocity=1.5,
        lower=-math.radians(55.0),
        upper=math.radians(75.0),
    )
    model.articulation(
        "bar_to_left_head",
        ArticulationType.REVOLUTE,
        parent=rotating_bar,
        child="left_head",
        origin=Origin(xyz=(-0.173, 0.0, 0.172)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tilt_limits,
    )
    model.articulation(
        "bar_to_right_head",
        ArticulationType.REVOLUTE,
        parent=rotating_bar,
        child="right_head",
        origin=Origin(xyz=(0.173, 0.0, 0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=tilt_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    ceiling_plate = object_model.get_part("ceiling_plate")
    rotating_bar = object_model.get_part("rotating_bar")
    left_head = object_model.get_part("left_head")
    right_head = object_model.get_part("right_head")

    plate_to_bar = object_model.get_articulation("plate_to_bar")
    left_tilt = object_model.get_articulation("bar_to_left_head")
    right_tilt = object_model.get_articulation("bar_to_right_head")

    ctx.expect_contact(
        ceiling_plate,
        rotating_bar,
        elem_a="plate_socket",
        elem_b="swivel_stem",
        contact_tol=1e-5,
        name="rotating stem seats into the ceiling socket",
    )
    ctx.expect_contact(
        rotating_bar,
        left_head,
        elem_a="left_inner_yoke",
        elem_b="hinge_trunnion",
        contact_tol=1e-5,
        name="left head is supported by the yoke at the hinge",
    )
    ctx.expect_contact(
        rotating_bar,
        right_head,
        elem_a="right_inner_yoke",
        elem_b="hinge_trunnion",
        contact_tol=1e-5,
        name="right head is supported by the yoke at the hinge",
    )
    ctx.expect_origin_distance(
        left_head,
        right_head,
        axes="x",
        min_dist=0.32,
        name="spot heads sit on opposite arms",
    )

    right_rest = ctx.part_world_position(right_head)
    with ctx.pose({plate_to_bar: math.radians(50.0)}):
        right_rotated = ctx.part_world_position(right_head)
    ctx.check(
        "central rod rotates the branching bar around the ceiling plate",
        right_rest is not None
        and right_rotated is not None
        and right_rotated[1] > right_rest[1] + 0.10,
        details=f"rest={right_rest}, rotated={right_rotated}",
    )

    right_shell_rest = ctx.part_element_world_aabb(right_head, elem="front_bezel")
    with ctx.pose({right_tilt: math.radians(50.0)}):
        right_shell_tilted = ctx.part_element_world_aabb(right_head, elem="front_bezel")
    ctx.check(
        "right spot head tilts downward on its hinge",
        right_shell_rest is not None
        and right_shell_tilted is not None
        and right_shell_tilted[0][2] < right_shell_rest[0][2] - 0.02,
        details=f"rest={right_shell_rest}, tilted={right_shell_tilted}",
    )

    left_shell_rest = ctx.part_element_world_aabb(left_head, elem="front_bezel")
    with ctx.pose({left_tilt: math.radians(45.0)}):
        left_shell_tilted = ctx.part_element_world_aabb(left_head, elem="front_bezel")
    ctx.check(
        "left spot head also tilts downward on its hinge",
        left_shell_rest is not None
        and left_shell_tilted is not None
        and left_shell_tilted[0][2] < left_shell_rest[0][2] - 0.02,
        details=f"rest={left_shell_rest}, tilted={left_shell_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
