from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner_pen")

    barrel_black = model.material("barrel_black", rgba=(0.14, 0.15, 0.17, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.12, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.82, 0.84, 0.87, 1.0))
    spinner_blue = model.material("spinner_blue", rgba=(0.16, 0.34, 0.72, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.70, 0.73, 0.77, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.0061, length=0.118),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_black,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.0068, length=0.015),
        origin=Origin(xyz=(-0.0665, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_black,
        name="rear_cap",
    )
    barrel.visual(
        Cylinder(radius=0.0069, length=0.028),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="grip_sleeve",
    )
    barrel.visual(
        Cylinder(radius=0.0080, length=0.0016),
        origin=Origin(xyz=(-0.0039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_silver,
        name="retainer_0",
    )
    barrel.visual(
        Cylinder(radius=0.0080, length=0.0016),
        origin=Origin(xyz=(0.0039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_silver,
        name="retainer_1",
    )

    tip_cone = _mesh(
        "tip_cone",
        ConeGeometry(radius=0.0039, height=0.018, radial_segments=40, closed=True).rotate_y(math.pi / 2.0),
    )
    barrel.visual(
        tip_cone,
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        material=metal_silver,
        name="tip_cone",
    )
    barrel.visual(
        Cylinder(radius=0.0011, length=0.0042),
        origin=Origin(xyz=(0.0791, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_silver,
        name="tip_nib",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.153, 0.016, 0.016)),
        mass=0.018,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    spinner = model.part("spinner")
    wing_plate = _mesh(
        "wing_plate",
        ExtrudeGeometry(
            rounded_rect_profile(0.021, 0.046, 0.009, corner_segments=8),
            height=0.0062,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    spinner.visual(
        wing_plate,
        origin=Origin(xyz=(0.0, 0.0295, 0.0)),
        material=spinner_blue,
        name="wing_0",
    )
    spinner.visual(
        wing_plate,
        origin=Origin(xyz=(0.0, -0.0295, 0.0)),
        material=spinner_blue,
        name="wing_1",
    )

    hub_shell = _mesh(
        "hub_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0115, -0.0031),
                (0.0122, -0.0019),
                (0.0128, 0.0),
                (0.0122, 0.0019),
                (0.0115, 0.0031),
            ],
            [
                (0.00685, -0.0031),
                (0.00685, 0.0031),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )
    spinner.visual(
        hub_shell,
        material=accent_silver,
        name="hub_shell",
    )
    spinner.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_silver,
        name="weight_0",
    )
    spinner.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_silver,
        name="weight_1",
    )
    spinner.inertial = Inertial.from_geometry(
        Box((0.011, 0.092, 0.030)),
        mass=0.028,
        origin=Origin(),
    )

    model.articulation(
        "barrel_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=spinner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    spinner = object_model.get_part("spinner")
    spinner_joint = object_model.get_articulation("barrel_to_spinner")

    ctx.expect_origin_distance(
        spinner,
        barrel,
        axes="yz",
        max_dist=1e-6,
        name="spinner shares the barrel axis",
    )
    ctx.expect_overlap(
        barrel,
        spinner,
        axes="yz",
        elem_a="barrel_shell",
        elem_b="hub_shell",
        min_overlap=0.012,
        name="spinner hub stays centered over the barrel",
    )
    ctx.expect_gap(
        barrel,
        spinner,
        axis="x",
        positive_elem="tip_nib",
        min_gap=0.060,
        name="ballpoint tip projects ahead of the spinner",
    )
    ctx.expect_contact(
        barrel,
        spinner,
        elem_a="retainer_0",
        elem_b="hub_shell",
        name="spinner hub bears against the rear retainer",
    )
    ctx.expect_contact(
        barrel,
        spinner,
        elem_a="retainer_1",
        elem_b="hub_shell",
        name="spinner hub bears against the front retainer",
    )

    with ctx.pose({spinner_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(spinner)
    with ctx.pose({spinner_joint: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_world_aabb(spinner)

    rest_dims = None
    quarter_turn_dims = None
    if rest_aabb is not None:
        rest_dims = tuple(rest_aabb[1][i] - rest_aabb[0][i] for i in range(3))
    if quarter_turn_aabb is not None:
        quarter_turn_dims = tuple(quarter_turn_aabb[1][i] - quarter_turn_aabb[0][i] for i in range(3))

    ctx.check(
        "spinner turns about the pen axis",
        rest_dims is not None
        and quarter_turn_dims is not None
        and rest_dims[1] > rest_dims[2] + 0.030
        and quarter_turn_dims[2] > quarter_turn_dims[1] + 0.030
        and abs(rest_dims[0] - quarter_turn_dims[0]) < 0.002,
        details=f"rest_dims={rest_dims}, quarter_turn_dims={quarter_turn_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
