from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


RAIL_LENGTH = 1.48
RAIL_WIDTH = 0.065
RAIL_HEIGHT = 0.040
GRIP_HEIGHT = 0.100
DROP_TRAVEL = 0.060
INSERT_LENGTH = 0.085
CORD_LENGTH = 0.420
CORD_RADIUS = 0.0018
INSERT_RADIUS = 0.0028
SOCKET_RADIUS = 0.0078
SOCKET_LENGTH = 0.024
SHADE_HEIGHT = 0.170
SHADE_MAX_RADIUS = 0.088


def _pendant_x_positions() -> list[float]:
    return [-0.56, -0.28, 0.0, 0.28, 0.56]


def _grip_barrel_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0100, 0.000),
            (0.0104, 0.055),
            (0.0110, 0.084),
            (0.0115, 0.100),
        ],
        [
            (0.0064, 0.000),
            (0.0068, 0.055),
            (0.0072, 0.084),
            (0.0076, 0.100),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )


def _shade_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.028, -0.012),
            (0.048, -0.040),
            (0.072, -0.102),
            (0.088, -0.170),
        ],
        [
            (0.014, 0.000),
            (0.023, -0.012),
            (0.042, -0.040),
            (0.065, -0.102),
            (0.080, -0.164),
        ],
        segments=60,
        start_cap="flat",
        end_cap="flat",
    )


def _shade_hanger_mesh():
    return LatheGeometry(
        [
            (0.0000, 0.000),
            (0.0020, -0.008),
            (0.0075, -0.016),
            (0.0145, -0.024),
            (0.0170, -0.034),
            (0.0160, -0.044),
        ],
        segments=40,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_island_multi_pendant_track")

    rail_finish = model.material("rail_finish", rgba=(0.63, 0.58, 0.51, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.13, 0.13, 0.14, 1.0))
    warm_white = model.material("warm_white", rgba=(0.96, 0.94, 0.88, 1.0))
    smoke_glass = model.material("smoke_glass", rgba=(0.42, 0.43, 0.46, 0.55))

    grip_barrel = mesh_from_geometry(_grip_barrel_mesh(), "grip_barrel")
    shade_hanger = mesh_from_geometry(_shade_hanger_mesh(), "pendant_shade_hanger")
    shade_shell = mesh_from_geometry(_shade_shell_mesh(), "pendant_shade_shell")

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * RAIL_HEIGHT)),
        material=rail_finish,
        name="rail_body",
    )
    rail.visual(
        Box((RAIL_LENGTH * 0.86, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -RAIL_HEIGHT + 0.003)),
        material=dark_metal,
        name="underside_channel",
    )
    rail.visual(
        Box((0.160, 0.048, 0.008)),
        origin=Origin(xyz=(-0.42, 0.0, -0.004)),
        material=rail_finish,
        name="mount_pad_left",
    )
    rail.visual(
        Box((0.160, 0.048, 0.008)),
        origin=Origin(xyz=(0.42, 0.0, -0.004)),
        material=rail_finish,
        name="mount_pad_right",
    )

    grip_opening_z = -RAIL_HEIGHT - GRIP_HEIGHT
    for index, x_pos in enumerate(_pendant_x_positions(), start=1):
        rail.visual(
            grip_barrel,
            origin=Origin(xyz=(x_pos, 0.0, grip_opening_z)),
            material=dark_metal,
            name=f"grip_barrel_{index}",
        )

    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * RAIL_HEIGHT)),
    )

    for index, x_pos in enumerate(_pendant_x_positions(), start=1):
        drop = model.part(f"drop_{index}")
        drop.visual(
            Cylinder(radius=INSERT_RADIUS, length=INSERT_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, 0.5 * INSERT_LENGTH)),
            material=dark_metal,
            name="upper_insert",
        )
        drop.visual(
            Cylinder(radius=0.0086, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, -0.0035)),
            material=dark_metal,
            name="grip_stop",
        )
        drop.visual(
            Cylinder(radius=CORD_RADIUS, length=CORD_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, -0.5 * CORD_LENGTH)),
            material=dark_metal,
            name="cord_segment",
        )
        drop.visual(
            Cylinder(radius=SOCKET_RADIUS, length=SOCKET_LENGTH),
            origin=Origin(
                xyz=(0.0, 0.0, -CORD_LENGTH - 0.5 * SOCKET_LENGTH),
            ),
            material=dark_metal,
            name="lower_socket",
        )
        drop.visual(
            Cylinder(radius=0.0046, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -CORD_LENGTH - SOCKET_LENGTH - 0.007)),
            material=dark_metal,
            name="pivot_stem",
        )
        drop.inertial = Inertial.from_geometry(
            Box((0.030, 0.030, 0.550)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, -0.170)),
        )

        model.articulation(
            f"rail_to_drop_{index}",
            ArticulationType.PRISMATIC,
            parent=rail,
            child=drop,
            origin=Origin(xyz=(x_pos, 0.0, grip_opening_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.10,
                lower=0.0,
                upper=DROP_TRAVEL,
            ),
        )

        shade = model.part(f"shade_{index}")
        shade.visual(
            shade_hanger,
            origin=Origin(),
            material=dark_metal,
            name="shade_hanger",
        )
        shade.visual(
            shade_shell,
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
            material=smoke_glass,
            name="shade_shell",
        )
        shade.visual(
            Cylinder(radius=0.016, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
            material=dark_metal,
            name="lamp_socket",
        )
        shade.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.074)),
            material=dark_metal,
            name="bulb_stem",
        )
        shade.visual(
            Sphere(radius=0.031),
            origin=Origin(xyz=(0.0, 0.0, -0.112)),
            material=warm_white,
            name="bulb_globe",
        )
        shade.inertial = Inertial.from_geometry(
            Cylinder(radius=SHADE_MAX_RADIUS, length=SHADE_HEIGHT + 0.050),
            mass=0.42,
            origin=Origin(xyz=(0.0, 0.0, -0.100)),
        )

        model.articulation(
            f"drop_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=drop,
            child=shade,
            origin=Origin(xyz=(0.0, 0.0, -CORD_LENGTH - SOCKET_LENGTH - 0.014)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.0,
                lower=-0.55,
                upper=0.55,
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
    rail = object_model.get_part("rail")
    rail_aabb = ctx.part_world_aabb(rail)
    ctx.check(
        "rail sits flush to the ceiling plane",
        rail_aabb is not None and abs(rail_aabb[1][2]) <= 1e-6,
        details=f"rail_aabb={rail_aabb}",
    )

    for index in range(1, 6):
        drop = object_model.get_part(f"drop_{index}")
        shade = object_model.get_part(f"shade_{index}")
        drop_joint = object_model.get_articulation(f"rail_to_drop_{index}")
        shade_joint = object_model.get_articulation(f"drop_to_shade_{index}")

        upper_drop = drop_joint.motion_limits.upper if drop_joint.motion_limits else None
        upper_tilt = shade_joint.motion_limits.upper if shade_joint.motion_limits else None

        ctx.expect_overlap(
            drop,
            rail,
            axes="z",
            elem_a="upper_insert",
            elem_b=f"grip_barrel_{index}",
            min_overlap=0.070,
            name=f"drop {index} insert stays captured at rest",
        )

        rest_shade_pos = ctx.part_world_position(shade)
        with ctx.pose({drop_joint: upper_drop}):
            ctx.expect_overlap(
                drop,
                rail,
                axes="z",
                elem_a="upper_insert",
                elem_b=f"grip_barrel_{index}",
                min_overlap=0.020,
                name=f"drop {index} insert remains captured at max extension",
            )
            extended_shade_pos = ctx.part_world_position(shade)

        ctx.check(
            f"drop {index} lowers the pendant when extended",
            rest_shade_pos is not None
            and extended_shade_pos is not None
            and extended_shade_pos[2] < rest_shade_pos[2] - 0.050,
            details=f"rest={rest_shade_pos}, extended={extended_shade_pos}",
        )

        rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        with ctx.pose({shade_joint: upper_tilt}):
            tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

        rest_center = None
        tilted_center = None
        if rest_shell_aabb is not None:
            rest_center = (
                0.5 * (rest_shell_aabb[0][0] + rest_shell_aabb[1][0]),
                0.5 * (rest_shell_aabb[0][2] + rest_shell_aabb[1][2]),
            )
        if tilted_shell_aabb is not None:
            tilted_center = (
                0.5 * (tilted_shell_aabb[0][0] + tilted_shell_aabb[1][0]),
                0.5 * (tilted_shell_aabb[0][2] + tilted_shell_aabb[1][2]),
            )

        ctx.check(
            f"shade {index} tilts forward on its y-axis hinge",
            rest_center is not None
            and tilted_center is not None
            and tilted_center[0] < rest_center[0] - 0.025
            and tilted_center[1] > rest_center[1] + 0.010,
            details=f"rest_center={rest_center}, tilted_center={tilted_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
