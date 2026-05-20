from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.180
HANDLE_WIDTH = 0.022
HANDLE_HEIGHT = 0.038


def _translated_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _make_handle_mesh() -> MeshGeometry:
    side_profile = [
        (-0.083, 0.000),
        (-0.030, 0.000),
        (0.050, 0.003),
        (0.070, 0.006),
        (0.084, 0.014),
        (0.090, 0.030),
        (0.081, 0.038),
        (-0.060, 0.038),
        (-0.082, 0.032),
        (-0.090, 0.004),
    ]

    side_plate = ExtrudeGeometry.centered(side_profile, 0.0024, cap=True, closed=True)
    side_plate.rotate_x(math.pi / 2.0)

    handle_geom = MeshGeometry()
    handle_geom.merge(side_plate.copy().translate(0.0, -0.0098, 0.0))
    handle_geom.merge(side_plate.copy().translate(0.0, 0.0098, 0.0))
    handle_geom.merge(_translated_box((0.152, 0.0176, 0.0044), (-0.005, 0.0, 0.0022)))
    handle_geom.merge(_translated_box((0.016, 0.0176, 0.030), (-0.082, 0.0, 0.019)))
    handle_geom.merge(_translated_box((0.148, 0.0046, 0.0046), (-0.006, -0.0067, 0.0352)))
    handle_geom.merge(_translated_box((0.148, 0.0046, 0.0046), (-0.006, 0.0067, 0.0352)))
    handle_geom.merge(_translated_box((0.026, 0.0176, 0.0060), (0.076, 0.0, 0.0030)))

    pivot_boss = CylinderGeometry(radius=0.0052, height=0.0026, radial_segments=24)
    pivot_boss.rotate_x(math.pi / 2.0).translate(-0.004, 0.0097, 0.020)
    handle_geom.merge(pivot_boss)
    return handle_geom


def _make_carriage_mesh() -> MeshGeometry:
    carriage_geom = MeshGeometry()
    carriage_geom.merge(_translated_box((0.102, 0.0126, 0.0120), (0.051, 0.0, 0.013)))
    carriage_geom.merge(_translated_box((0.020, 0.0126, 0.0160), (0.104, 0.0, 0.017)))
    carriage_geom.merge(_translated_box((0.010, 0.0060, 0.0110), (0.058, 0.0, 0.0215)))
    carriage_geom.merge(_translated_box((0.024, 0.0088, 0.0080), (0.058, 0.0, 0.0305)))
    return carriage_geom


def _make_blade_mesh() -> MeshGeometry:
    blade_profile = [
        (0.000, 0.000),
        (0.000, 0.018),
        (0.022, 0.018),
        (0.038, 0.012),
        (0.026, 0.000),
    ]
    blade_geom = ExtrudeGeometry.centered(blade_profile, 0.0009, cap=True, closed=True)
    blade_geom.rotate_x(math.pi / 2.0)
    blade_geom.translate(0.095, 0.0, 0.009)
    return blade_geom


def _make_lock_lever_mesh() -> MeshGeometry:
    lever_geom = MeshGeometry()

    hub = CylinderGeometry(radius=0.0042, height=0.0036, radial_segments=24)
    hub.rotate_x(math.pi / 2.0).translate(0.0, 0.0018, 0.0)
    lever_geom.merge(hub)
    lever_geom.merge(_translated_box((0.024, 0.0036, 0.0060), (0.012, 0.0018, -0.001)))
    lever_geom.merge(_translated_box((0.008, 0.0036, 0.0100), (0.025, 0.0018, -0.001)))
    return lever_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_knife")

    handle_black = model.material("handle_black", rgba=(0.16, 0.16, 0.17, 1.0))
    carriage_graphite = model.material("carriage_graphite", rgba=(0.30, 0.31, 0.33, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    lever_orange = model.material("lever_orange", rgba=(0.90, 0.46, 0.12, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(_make_handle_mesh(), "utility_knife_handle"),
        material=handle_black,
        name="handle_shell",
    )

    carriage = model.part("blade_carriage")
    carriage.visual(
        mesh_from_geometry(_make_carriage_mesh(), "utility_knife_carriage"),
        material=carriage_graphite,
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_geometry(_make_blade_mesh(), "utility_knife_blade"),
        material=blade_steel,
        name="blade",
    )

    lock_lever = model.part("blade_lock_lever")
    lock_lever.visual(
        mesh_from_geometry(_make_lock_lever_mesh(), "utility_knife_lock_lever"),
        material=lever_orange,
        name="lock_lever",
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=0.048,
        ),
    )

    model.articulation(
        "handle_to_blade_lock_lever",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=lock_lever,
        origin=Origin(xyz=(-0.004, 0.0110, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.10,
            upper=0.65,
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

    handle = object_model.get_part("handle")
    carriage = object_model.get_part("blade_carriage")
    lock_lever = object_model.get_part("blade_lock_lever")

    slide = object_model.get_articulation("handle_to_blade_carriage")
    lever_joint = object_model.get_articulation("handle_to_blade_lock_lever")

    handle_shell = handle.get_visual("handle_shell")
    carriage_body = carriage.get_visual("carriage_body")
    blade = carriage.get_visual("blade")

    slide_upper = slide.motion_limits.upper if slide.motion_limits and slide.motion_limits.upper is not None else 0.048
    lever_upper = (
        lever_joint.motion_limits.upper
        if lever_joint.motion_limits and lever_joint.motion_limits.upper is not None
        else 0.65
    )

    ctx.check(
        "blade carriage slides along handle axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "lock lever pivots on side-facing y axis",
        tuple(lever_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lever_joint.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            carriage,
            handle,
            axes="yz",
            inner_elem=carriage_body,
            outer_elem=handle_shell,
            margin=0.0,
            name="carriage body sits inside the handle channel at rest",
        )
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a=carriage_body,
            elem_b=handle_shell,
            min_overlap=0.090,
            name="retracted carriage remains deeply inserted in the handle",
        )

    handle_aabb = ctx.part_world_aabb(handle)
    blade_rest_aabb = ctx.part_element_world_aabb(carriage, elem=blade)
    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_lever_aabb = ctx.part_world_aabb(lock_lever)

    with ctx.pose({slide: slide_upper}):
        ctx.expect_within(
            carriage,
            handle,
            axes="yz",
            inner_elem=carriage_body,
            outer_elem=handle_shell,
            margin=0.0,
            name="extended carriage body still rides inside the handle channel",
        )
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a=carriage_body,
            elem_b=handle_shell,
            min_overlap=0.075,
            name="extended carriage keeps retained insertion inside the handle",
        )
        blade_extended_aabb = ctx.part_element_world_aabb(carriage, elem=blade)
        extended_carriage_pos = ctx.part_world_position(carriage)

    with ctx.pose({lever_joint: lever_upper}):
        open_lever_aabb = ctx.part_world_aabb(lock_lever)

    ctx.check(
        "blade stays nearly flush when retracted",
        handle_aabb is not None
        and blade_rest_aabb is not None
        and blade_rest_aabb[1][0] <= handle_aabb[1][0] + 0.006,
        details=f"handle={handle_aabb}, blade_rest={blade_rest_aabb}",
    )
    ctx.check(
        "blade projects forward when carriage extends",
        handle_aabb is not None
        and blade_extended_aabb is not None
        and blade_extended_aabb[1][0] >= handle_aabb[1][0] + 0.025,
        details=f"handle={handle_aabb}, blade_extended={blade_extended_aabb}",
    )
    ctx.check(
        "carriage moves forward under positive travel",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] >= rest_carriage_pos[0] + 0.040,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "lock lever lifts when opened",
        rest_lever_aabb is not None
        and open_lever_aabb is not None
        and open_lever_aabb[1][2] >= rest_lever_aabb[1][2] + 0.010,
        details=f"rest={rest_lever_aabb}, open={open_lever_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
