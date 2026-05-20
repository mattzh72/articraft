from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def midpoint(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def distance(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def rpy_for_cylinder(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def add_member(part, name: str, a, b, radius: float, material) -> None:
        part.visual(
            Cylinder(radius=radius, length=distance(a, b)),
            origin=Origin(xyz=midpoint(a, b), rpy=rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    model = ArticulatedObject(name="swivel_bar_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    base_shell = save_mesh(
        "base_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.060, 0.0),
                (0.155, 0.003),
                (0.198, 0.010),
                (0.213, 0.018),
                (0.215, 0.026),
                (0.170, 0.030),
                (0.090, 0.031),
                (0.0, 0.031),
            ],
            segments=64,
        ),
    )
    seat_bracket = save_mesh(
        "seat_bracket",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.038, 0.0),
                (0.054, 0.010),
                (0.080, 0.025),
                (0.098, 0.032),
                (0.102, 0.034),
                (0.0, 0.034),
            ],
            segments=56,
        ),
    )
    seat_cushion = save_mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.090, 0.0),
                (0.145, 0.008),
                (0.168, 0.024),
                (0.177, 0.042),
                (0.166, 0.055),
                (0.0, 0.058),
            ],
            segments=64,
        ),
    )
    foot_ring = save_mesh(
        "foot_ring",
        TorusGeometry(radius=0.190, tube=0.012, radial_segments=18, tubular_segments=72),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(base_shell, material=brushed_steel, name="floor_base_shell")
    pedestal_base.visual(
        Cylinder(radius=0.188, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="floor_pad",
    )
    pedestal_base.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=brushed_steel,
        name="lower_shroud",
    )
    pedestal_base.visual(
        Cylinder(radius=0.045, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=brushed_steel,
        name="seat_post",
    )
    pedestal_base.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_steel,
        name="foot_ring_collar",
    )
    pedestal_base.visual(
        foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_steel,
        name="foot_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        inner = (math.cos(angle) * 0.044, math.sin(angle) * 0.044, 0.322)
        outer = (math.cos(angle) * 0.181, math.sin(angle) * 0.181, 0.310)
        add_member(
            pedestal_base,
            f"foot_ring_brace_{index}",
            inner,
            outer,
            radius=0.009,
            material=dark_steel,
        )
    pedestal_base.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=brushed_steel,
        name="upper_shroud",
    )
    pedestal_base.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=dark_steel,
        name="upper_bearing_housing",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.650),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_steel,
        name="swivel_stem",
    )
    seat.visual(
        seat_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_steel,
        name="seat_mount_hub",
    )
    seat.visual(
        Cylinder(radius=0.158, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_steel,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=black_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.177, length=0.122),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.check(
        "seat uses a continuous swivel joint",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={swivel.articulation_type}",
    )
    ctx.check(
        "seat swivel axis is vertical",
        swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.expect_gap(
        seat,
        pedestal_base,
        axis="z",
        positive_elem="swivel_stem",
        negative_elem="upper_bearing_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel stem seats on the bearing housing",
    )
    ctx.expect_overlap(
        seat,
        pedestal_base,
        axes="xy",
        elem_a="swivel_stem",
        elem_b="upper_bearing_housing",
        min_overlap=0.050,
        name="rotating stem stays centered within the broader support",
    )
    ctx.expect_origin_gap(
        seat,
        pedestal_base,
        axis="z",
        min_gap=0.620,
        max_gap=0.680,
        name="seat sits at bar stool height above the base",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
