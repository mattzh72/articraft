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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_loft_section(
    x_pos: float,
    *,
    depth: float,
    height: float,
    radius: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y + y_center, z + z_center)
        for y, z in rounded_rect_profile(depth, height, radius, corner_segments=7)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_box")

    fascia = model.material("fascia", rgba=(0.19, 0.20, 0.22, 1.0))
    inner_liner = model.material("inner_liner", rgba=(0.11, 0.12, 0.13, 1.0))
    door_skin = model.material("door_skin", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.40, 0.43, 0.46, 1.0))
    handle_trim = model.material("handle_trim", rgba=(0.63, 0.66, 0.70, 1.0))

    housing = model.part("housing")

    upper_fascia_mesh = _save_mesh(
        "glovebox_upper_fascia",
        section_loft(
            [
                _yz_loft_section(
                    -0.250,
                    depth=0.048,
                    height=0.072,
                    radius=0.014,
                    y_center=0.006,
                    z_center=0.186,
                ),
                _yz_loft_section(
                    0.000,
                    depth=0.062,
                    height=0.072,
                    radius=0.018,
                    y_center=0.014,
                    z_center=0.186,
                ),
                _yz_loft_section(
                    0.250,
                    depth=0.048,
                    height=0.072,
                    radius=0.014,
                    y_center=0.006,
                    z_center=0.186,
                ),
            ]
        ),
    )
    housing.visual(upper_fascia_mesh, material=fascia, name="upper_fascia")
    housing.visual(
        Box((0.500, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, 0.002, -0.080)),
        material=fascia,
        name="lower_sill",
    )
    housing.visual(
        Box((0.068, 0.030, 0.240)),
        origin=Origin(xyz=(-0.216, 0.005, 0.035)),
        material=fascia,
        name="left_cheek",
    )
    housing.visual(
        Box((0.068, 0.030, 0.240)),
        origin=Origin(xyz=(0.216, 0.005, 0.035)),
        material=fascia,
        name="right_cheek",
    )

    housing.visual(
        Box((0.360, 0.230, 0.004)),
        origin=Origin(xyz=(0.0, -0.121, -0.062)),
        material=inner_liner,
        name="tub_floor",
    )
    housing.visual(
        Box((0.360, 0.004, 0.216)),
        origin=Origin(xyz=(0.0, -0.236, 0.046)),
        material=inner_liner,
        name="tub_back",
    )
    housing.visual(
        Box((0.360, 0.230, 0.004)),
        origin=Origin(xyz=(0.0, -0.121, 0.154)),
        material=inner_liner,
        name="tub_top",
    )
    housing.visual(
        Box((0.004, 0.230, 0.216)),
        origin=Origin(xyz=(-0.182, -0.121, 0.046)),
        material=inner_liner,
        name="left_tub_wall",
    )
    housing.visual(
        Box((0.004, 0.230, 0.216)),
        origin=Origin(xyz=(0.182, -0.121, 0.046)),
        material=inner_liner,
        name="right_tub_wall",
    )
    housing.visual(
        Box((0.040, 0.062, 0.056)),
        origin=Origin(xyz=(-0.194, -0.026, 0.070)),
        material=inner_liner,
        name="left_support_gusset",
    )
    housing.visual(
        Box((0.040, 0.062, 0.056)),
        origin=Origin(xyz=(0.194, -0.026, 0.070)),
        material=inner_liner,
        name="right_support_gusset",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.178, -0.012, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_support_boss",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.178, -0.012, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_support_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.500, 0.270, 0.250)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.105, 0.045)),
    )

    hinge_yoke = model.part("hinge_yoke")
    arm_angle = 0.514
    arm_length = 0.0715
    hinge_yoke.visual(
        Cylinder(radius=0.008, length=0.312),
        origin=Origin(xyz=(0.0, 0.010, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="cross_tube",
    )
    hinge_yoke.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(-0.164, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_support_sleeve",
    )
    hinge_yoke.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.164, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_support_sleeve",
    )
    hinge_yoke.visual(
        Box((0.016, 0.008, arm_length)),
        origin=Origin(xyz=(-0.164, 0.0175, -0.031), rpy=(arm_angle, 0.0, 0.0)),
        material=hinge_steel,
        name="left_arm",
    )
    hinge_yoke.visual(
        Box((0.016, 0.008, arm_length)),
        origin=Origin(xyz=(0.164, 0.0175, -0.031), rpy=(arm_angle, 0.0, 0.0)),
        material=hinge_steel,
        name="right_arm",
    )
    hinge_yoke.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.166, 0.035, -0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_lower_eye",
    )
    hinge_yoke.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.166, 0.035, -0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_lower_eye",
    )
    hinge_yoke.inertial = Inertial.from_geometry(
        Box((0.370, 0.085, 0.090)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.018, -0.016)),
    )

    door = model.part("door")
    door_shell_mesh = _save_mesh(
        "glovebox_door_shell",
        section_loft(
            [
                _yz_loft_section(
                    -0.157,
                    depth=0.032,
                    height=0.138,
                    radius=0.012,
                    y_center=0.006,
                    z_center=0.072,
                ),
                _yz_loft_section(
                    0.000,
                    depth=0.040,
                    height=0.138,
                    radius=0.014,
                    y_center=0.012,
                    z_center=0.072,
                ),
                _yz_loft_section(
                    0.157,
                    depth=0.032,
                    height=0.138,
                    radius=0.012,
                    y_center=0.006,
                    z_center=0.072,
                ),
            ]
        ),
    )
    door.visual(door_shell_mesh, material=door_skin, name="door_shell")
    door.visual(
        Box((0.292, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, -0.044, 0.132)),
        material=inner_liner,
        name="tray_top_wall",
    )
    door.visual(
        Box((0.012, 0.032, 0.050)),
        origin=Origin(xyz=(-0.150, -0.044, 0.110)),
        material=inner_liner,
        name="left_tray_wall",
    )
    door.visual(
        Box((0.012, 0.032, 0.050)),
        origin=Origin(xyz=(0.150, -0.044, 0.110)),
        material=inner_liner,
        name="right_tray_wall",
    )
    door.visual(
        Box((0.292, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.008)),
        material=inner_liner,
        name="lower_inner_lip",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_pivot_boss",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_pivot_boss",
    )
    door.visual(
        Box((0.292, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, 0.122)),
        material=inner_liner,
        name="tray_bridge",
    )
    door.visual(
        Box((0.012, 0.020, 0.030)),
        origin=Origin(xyz=(-0.150, -0.018, 0.098)),
        material=inner_liner,
        name="left_tray_brace",
    )
    door.visual(
        Box((0.012, 0.020, 0.030)),
        origin=Origin(xyz=(0.150, -0.018, 0.098)),
        material=inner_liner,
        name="right_tray_brace",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(-0.072, 0.034, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_handle_boss",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.072, 0.034, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_handle_boss",
    )
    door.visual(
        Box((0.190, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, 0.070)),
        material=fascia,
        name="handle_escutcheon",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.340, 0.075, 0.150)),
        mass=0.95,
        origin=Origin(xyz=(0.0, -0.014, 0.075)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(-0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_handle_stub",
    )
    handle.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_handle_stub",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.028),
        origin=Origin(xyz=(-0.056, 0.010, -0.014)),
        material=handle_trim,
        name="left_handle_leg",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.028),
        origin=Origin(xyz=(0.056, 0.010, -0.014)),
        material=handle_trim,
        name="right_handle_leg",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.152),
        origin=Origin(xyz=(0.0, 0.018, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_trim,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.170, 0.030, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.014, -0.016)),
    )

    model.articulation(
        "housing_to_hinge_yoke",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=hinge_yoke,
        origin=Origin(xyz=(0.0, -0.012, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=0.68,
        ),
    )
    model.articulation(
        "hinge_yoke_to_door",
        ArticulationType.REVOLUTE,
        parent=hinge_yoke,
        child=door,
        origin=Origin(xyz=(0.0, 0.035, -0.062)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=2.18,
        ),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.0, 0.034, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=0.0,
            upper=0.38,
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

    housing = object_model.get_part("housing")
    hinge_yoke = object_model.get_part("hinge_yoke")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")

    yoke_joint = object_model.get_articulation("housing_to_hinge_yoke")
    door_joint = object_model.get_articulation("hinge_yoke_to_door")
    handle_joint = object_model.get_articulation("door_to_handle")

    ctx.check(
        "all glove box parts exist",
        all(part is not None for part in (housing, hinge_yoke, door, handle)),
    )

    with ctx.pose({yoke_joint: 0.0, door_joint: 0.0, handle_joint: 0.0}):
        ctx.expect_contact(
            hinge_yoke,
            housing,
            elem_a="left_support_sleeve",
            elem_b="left_support_boss",
            name="left hinge arm support joint is seated",
        )
        ctx.expect_contact(
            hinge_yoke,
            housing,
            elem_a="right_support_sleeve",
            elem_b="right_support_boss",
            name="right hinge arm support joint is seated",
        )
        ctx.expect_contact(
            door,
            hinge_yoke,
            elem_a="left_pivot_boss",
            elem_b="left_lower_eye",
            name="left lower door pivot is seated",
        )
        ctx.expect_contact(
            door,
            hinge_yoke,
            elem_a="right_pivot_boss",
            elem_b="right_lower_eye",
            name="right lower door pivot is seated",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="left_handle_stub",
            elem_b="left_handle_boss",
            name="left handle trunnion is seated",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="right_handle_stub",
            elem_b="right_handle_boss",
            name="right handle trunnion is seated",
        )

        rest_eye_center = _aabb_center(ctx.part_element_world_aabb(hinge_yoke, elem="left_lower_eye"))
        rest_door_center = _aabb_center(ctx.part_element_world_aabb(door, elem="door_shell"))
        rest_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))

    with ctx.pose({yoke_joint: 0.56, door_joint: 2.04, handle_joint: 0.30}):
        open_eye_center = _aabb_center(ctx.part_element_world_aabb(hinge_yoke, elem="left_lower_eye"))
        open_door_center = _aabb_center(ctx.part_element_world_aabb(door, elem="door_shell"))
        open_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))

        ctx.check(
            "hinge arms swing their lower pivots outward",
            rest_eye_center is not None
            and open_eye_center is not None
            and open_eye_center[1] > rest_eye_center[1] + 0.018
            and open_eye_center[2] > rest_eye_center[2] + 0.020,
            details=f"rest={rest_eye_center}, open={open_eye_center}",
        )
        ctx.check(
            "door drops down and outward into a tray pose",
            rest_door_center is not None
            and open_door_center is not None
            and open_door_center[1] > rest_door_center[1] + 0.070
            and open_door_center[2] < rest_door_center[2] - 0.040,
            details=f"rest={rest_door_center}, open={open_door_center}",
        )
        ctx.check(
            "pull handle rotates outward from the door face",
            rest_handle_center is not None
            and open_handle_center is not None
            and open_handle_center[1] > rest_handle_center[1] + 0.004,
            details=f"rest={rest_handle_center}, open={open_handle_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
