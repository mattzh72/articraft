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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _handle_side_profile() -> list[tuple[float, float]]:
    return [
        (-0.092, 0.004),
        (-0.090, 0.030),
        (-0.070, 0.033),
        (-0.028, 0.035),
        (0.016, 0.033),
        (0.052, 0.028),
        (0.092, 0.021),
        (0.086, 0.012),
        (0.054, 0.006),
        (0.006, 0.003),
        (-0.040, 0.001),
        (-0.078, 0.002),
    ]


def _blade_profile() -> list[tuple[float, float]]:
    return [
        (-0.030, -0.0065),
        (0.010, -0.0065),
        (0.029, 0.0000),
        (0.010, 0.0075),
        (-0.015, 0.0075),
        (-0.030, 0.0015),
    ]


def _build_side_shell_mesh():
    geom = ExtrudeGeometry(_handle_side_profile(), 0.004, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "utility_knife_side_shell")


def _build_blade_mesh():
    geom = ExtrudeGeometry(_blade_profile(), 0.0008, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "utility_knife_blade_blank")


def _build_ring_loop_mesh():
    geom = wire_from_points(
        [
            (0.000, 0.000, 0.000),
            (-0.018, 0.000, 0.001),
            (-0.036, 0.000, -0.006),
            (-0.041, 0.000, -0.018),
            (-0.030, 0.000, -0.031),
            (-0.006, 0.000, -0.032),
            (0.006, 0.000, -0.019),
            (0.004, 0.000, -0.005),
        ],
        radius=0.0022,
        radial_segments=18,
        closed_path=True,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.006,
        corner_segments=10,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, "utility_knife_lanyard_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contractor_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.88, 0.76, 0.17, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.44, 0.46, 0.49, 1.0))

    side_shell_mesh = _build_side_shell_mesh()
    blade_mesh = _build_blade_mesh()
    ring_loop_mesh = _build_ring_loop_mesh()

    handle = model.part("handle_body")
    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=body_yellow,
        name="left_shell",
    )
    handle.visual(
        side_shell_mesh,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=body_yellow,
        name="right_shell",
    )
    handle.visual(
        Box((0.132, 0.016, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.006)),
        material=dark_grey,
        name="bottom_spine",
    )
    handle.visual(
        Box((0.028, 0.016, 0.026)),
        origin=Origin(xyz=(-0.078, 0.0, 0.014)),
        material=dark_grey,
        name="rear_block",
    )
    handle.visual(
        Box((0.034, 0.016, 0.010)),
        origin=Origin(xyz=(0.071, 0.0, 0.006)),
        material=dark_grey,
        name="front_lower_nose",
    )
    handle.visual(
        Box((0.032, 0.016, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, 0.029)),
        material=dark_steel,
        name="front_upper_nose",
    )
    handle.visual(
        Box((0.106, 0.004, 0.007)),
        origin=Origin(xyz=(0.004, -0.006, 0.031)),
        material=dark_grey,
        name="left_top_rail",
    )
    handle.visual(
        Box((0.106, 0.004, 0.007)),
        origin=Origin(xyz=(0.004, 0.006, 0.031)),
        material=dark_grey,
        name="right_top_rail",
    )
    handle.visual(
        Box((0.026, 0.016, 0.008)),
        origin=Origin(xyz=(-0.079, 0.0, 0.031)),
        material=dark_grey,
        name="rear_top_cap",
    )
    handle.visual(
        Box((0.074, 0.0018, 0.017)),
        origin=Origin(xyz=(-0.014, -0.0129, 0.015)),
        material=rubber_black,
        name="left_grip_pad",
    )
    handle.visual(
        Box((0.074, 0.0018, 0.017)),
        origin=Origin(xyz=(-0.014, 0.0129, 0.015)),
        material=rubber_black,
        name="right_grip_pad",
    )
    handle.visual(
        Box((0.030, 0.0018, 0.018)),
        origin=Origin(xyz=(0.067, -0.0129, 0.018)),
        material=dark_steel,
        name="left_nose_insert",
    )
    handle.visual(
        Box((0.030, 0.0018, 0.018)),
        origin=Origin(xyz=(0.067, 0.0129, 0.018)),
        material=dark_steel,
        name="right_nose_insert",
    )
    handle.visual(
        Box((0.016, 0.005, 0.012)),
        origin=Origin(xyz=(-0.086, -0.008, 0.011)),
        material=dark_steel,
        name="left_ring_ear",
    )
    handle.visual(
        Box((0.016, 0.005, 0.012)),
        origin=Origin(xyz=(-0.086, 0.008, 0.011)),
        material=dark_steel,
        name="right_ring_ear",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.188, 0.026, 0.038)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    carriage = model.part("blade_carriage")
    carriage.visual(
        Box((0.104, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, -0.011)),
        material=dark_steel,
        name="carriage_runner",
    )
    carriage.visual(
        Box((0.038, 0.012, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, -0.010)),
        material=dark_steel,
        name="blade_clamp",
    )
    carriage.visual(
        blade_mesh,
        origin=Origin(xyz=(0.087, 0.0, -0.011)),
        material=steel,
        name="blade_blank",
    )
    carriage.visual(
        Box((0.006, 0.006, 0.018)),
        origin=Origin(xyz=(0.002, 0.0, -0.003)),
        material=dark_steel,
        name="slider_stem",
    )
    carriage.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(0.002, 0.0, 0.006)),
        material=dark_grey,
        name="thumb_slider",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.148, 0.018, 0.024)),
        mass=0.07,
        origin=Origin(xyz=(0.022, 0.0, -0.004)),
    )

    ring = model.part("lanyard_ring")
    ring.visual(
        ring_loop_mesh,
        material=steel,
        name="ring_loop",
    )
    ring.visual(
        Box((0.005, 0.007, 0.004)),
        origin=Origin(xyz=(-0.001, 0.0, -0.001)),
        material=dark_steel,
        name="hinge_yoke",
    )
    ring.visual(
        Cylinder(radius=0.0026, length=0.0038),
        origin=Origin(xyz=(0.000, -0.0032, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_ring_knuckle",
    )
    ring.visual(
        Cylinder(radius=0.0026, length=0.0038),
        origin=Origin(xyz=(0.000, 0.0032, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_ring_knuckle",
    )
    ring.inertial = Inertial.from_geometry(
        Box((0.050, 0.016, 0.042)),
        mass=0.015,
        origin=Origin(xyz=(-0.004, 0.0, -0.018)),
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(xyz=(-0.006, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
        ),
    )

    model.articulation(
        "handle_to_lanyard_ring",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=ring,
        origin=Origin(xyz=(-0.089, 0.0, 0.012)),
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
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    handle = object_model.get_part("handle_body")
    carriage = object_model.get_part("blade_carriage")
    ring = object_model.get_part("lanyard_ring")
    blade_slide = object_model.get_articulation("handle_to_blade_carriage")
    ring_hinge = object_model.get_articulation("handle_to_lanyard_ring")

    blade_upper = 0.055
    if blade_slide.motion_limits is not None and blade_slide.motion_limits.upper is not None:
        blade_upper = blade_slide.motion_limits.upper

    ring_upper = 1.45
    if ring_hinge.motion_limits is not None and ring_hinge.motion_limits.upper is not None:
        ring_upper = ring_hinge.motion_limits.upper

    ctx.expect_within(
        carriage,
        handle,
        axes="y",
        inner_elem="carriage_runner",
        outer_elem="bottom_spine",
        margin=0.0,
        name="carriage runner stays centered between the handle rails",
    )
    ctx.expect_overlap(
        carriage,
        handle,
        axes="x",
        elem_a="carriage_runner",
        elem_b="bottom_spine",
        min_overlap=0.090,
        name="retracted carriage runner sits deeply in the handle track",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({blade_slide: blade_upper}):
        ctx.expect_within(
            carriage,
            handle,
            axes="y",
            inner_elem="carriage_runner",
            outer_elem="bottom_spine",
            margin=0.0,
            name="extended carriage runner stays centered between the handle rails",
        )
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_runner",
            elem_b="bottom_spine",
            min_overlap=0.040,
            name="extended carriage runner retains insertion in the handle track",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "blade carriage extends forward along the handle axis",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.040,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))

    rest_ring_aabb = ctx.part_element_world_aabb(ring, elem="ring_loop")
    rest_ring_center = _aabb_center(rest_ring_aabb)
    with ctx.pose({ring_hinge: ring_upper}):
        open_ring_aabb = ctx.part_element_world_aabb(ring, elem="ring_loop")
        open_ring_center = _aabb_center(open_ring_aabb)
        rear_block_aabb = ctx.part_element_world_aabb(handle, elem="rear_block")

    rest_ring_low_z = None
    open_ring_low_z = None
    rear_block_low_z = None
    if rest_ring_aabb is not None:
        rest_ring_low_z = rest_ring_aabb[0][2]
    if open_ring_aabb is not None:
        open_ring_low_z = open_ring_aabb[0][2]
    if rear_block_aabb is not None:
        rear_block_low_z = rear_block_aabb[0][2]

    ctx.check(
        "deployed lanyard ring hangs below the tail block",
        open_ring_low_z is not None
        and rear_block_low_z is not None
        and open_ring_low_z < rear_block_low_z - 0.018,
        details=f"rear_block_low_z={rear_block_low_z}, open_ring_low_z={open_ring_low_z}",
    )

    ctx.check(
        "lanyard ring swings downward from the tail hinge",
        rest_ring_low_z is not None
        and open_ring_low_z is not None
        and open_ring_low_z < rest_ring_low_z - 0.008,
        details=(
            f"rest_center={rest_ring_center}, open_center={open_ring_center}, "
            f"rest_low_z={rest_ring_low_z}, open_low_z={open_ring_low_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
