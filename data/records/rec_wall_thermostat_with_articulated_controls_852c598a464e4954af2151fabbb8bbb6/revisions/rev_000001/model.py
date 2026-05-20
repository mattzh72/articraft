from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    rounded_rect_profile,
)


def _rounded_panel_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                width,
                height,
                radius,
                corner_segments=8,
            ),
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    plate_white = model.material("plate_white", rgba=(0.95, 0.95, 0.93, 1.0))
    shell_white = model.material("shell_white", rgba=(0.92, 0.92, 0.90, 1.0))
    bezel_warm = model.material("bezel_warm", rgba=(0.87, 0.87, 0.84, 1.0))
    ring_graphite = model.material("ring_graphite", rgba=(0.25, 0.27, 0.29, 1.0))
    face_gray = model.material("face_gray", rgba=(0.81, 0.82, 0.80, 1.0))
    metal_highlight = model.material("metal_highlight", rgba=(0.73, 0.75, 0.78, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.22, 0.24, 0.90))
    accent_dark = model.material("accent_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    accent_light = model.material("accent_light", rgba=(0.96, 0.96, 0.94, 1.0))

    wall_plate_mesh = _rounded_panel_mesh(
        "thermostat_wall_plate",
        width=0.118,
        height=0.178,
        thickness=0.004,
        radius=0.019,
    )
    body_shell_mesh = _rounded_panel_mesh(
        "thermostat_body_shell",
        width=0.094,
        height=0.154,
        thickness=0.019,
        radius=0.018,
    )
    front_bezel_mesh = _rounded_panel_mesh(
        "thermostat_front_bezel",
        width=0.086,
        height=0.146,
        thickness=0.005,
        radius=0.016,
    )
    status_window_mesh = _rounded_panel_mesh(
        "thermostat_status_window",
        width=0.028,
        height=0.011,
        thickness=0.0015,
        radius=0.0035,
    )

    housing = model.part("housing")
    housing.visual(
        wall_plate_mesh,
        material=plate_white,
        name="wall_plate",
    )
    housing.visual(
        body_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shell_white,
        name="body_shell",
    )
    housing.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=bezel_warm,
        name="body_front_bezel",
    )
    housing.visual(
        Cylinder(radius=0.058, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=bezel_warm,
        name="dial_cradle",
    )
    housing.visual(
        status_window_mesh,
        origin=Origin(xyz=(0.0, 0.058, 0.028)),
        material=smoked_glass,
        name="status_window",
    )
    housing.visual(
        Box((0.004, 0.010, 0.0016)),
        origin=Origin(xyz=(0.0, 0.044, 0.0288)),
        material=accent_dark,
        name="dial_reference",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.118, 0.178, 0.033)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.054, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=ring_graphite,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=face_gray,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=metal_highlight,
        name="dial_center_cap",
    )
    dial.visual(
        Box((0.005, 0.018, 0.0018)),
        origin=Origin(xyz=(0.0, 0.024, 0.0156)),
        material=accent_light,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.108, 0.108, 0.018)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    end_wheel = model.part("end_wheel")
    end_wheel.visual(
        Cylinder(radius=0.015, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=bezel_warm,
        name="end_wheel_collar",
    )
    end_wheel.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0062)),
        material=accent_dark,
        name="end_wheel",
    )
    end_wheel.visual(
        Box((0.003, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0085, 0.0091)),
        material=accent_light,
        name="end_wheel_marker",
    )
    end_wheel.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.011)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=6.0,
        ),
    )
    model.articulation(
        "housing_to_end_wheel",
        ArticulationType.FIXED,
        parent=housing,
        child=end_wheel,
        origin=Origin(xyz=(0.0, -0.073, 0.028)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    end_wheel = object_model.get_part("end_wheel")
    dial_joint = object_model.get_articulation("housing_to_dial")

    ctx.expect_origin_distance(
        dial,
        housing,
        axes="xy",
        max_dist=0.001,
        name="dial stays centered on the thermostat body",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="z",
        positive_elem="dial_ring",
        negative_elem="body_front_bezel",
        max_gap=0.001,
        max_penetration=1e-6,
        name="dial ring sits against the front bezel",
    )
    ctx.expect_overlap(
        dial,
        housing,
        axes="xy",
        elem_a="dial_ring",
        elem_b="dial_cradle",
        min_overlap=0.100,
        name="dial ring covers the central cradle",
    )
    ctx.expect_gap(
        end_wheel,
        housing,
        axis="z",
        positive_elem="end_wheel_collar",
        negative_elem="body_front_bezel",
        max_gap=0.001,
        max_penetration=1e-6,
        name="small end wheel mounts flush to the lower bezel",
    )

    rest_position = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.7}):
        turned_position = ctx.part_world_position(dial)
        ctx.expect_gap(
            dial,
            housing,
            axis="z",
            positive_elem="dial_ring",
            negative_elem="body_front_bezel",
            max_gap=0.001,
            max_penetration=1e-6,
            name="dial remains seated while rotated",
        )

    ctx.check(
        "dial rotates about a fixed center",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) <= 1e-6
        and abs(rest_position[1] - turned_position[1]) <= 1e-6
        and abs(rest_position[2] - turned_position[2]) <= 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
