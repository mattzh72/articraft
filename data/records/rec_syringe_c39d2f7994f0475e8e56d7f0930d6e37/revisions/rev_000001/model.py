from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


BARREL_INNER_RADIUS = 0.0085
BARREL_OUTER_RADIUS = 0.0105
BARREL_START_Z = 0.012
BARREL_LENGTH = 0.093
REAR_LIP_LENGTH = 0.006
GUIDE_RING_CENTER_Z = 0.113
GUIDE_RING_LENGTH = 0.006
PLUNGER_TRAVEL = 0.060


def _circle_profile(radius: float, *, segments: int = 56) -> list[tuple[float, float]]:
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def _annular_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            length,
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.85, 0.92, 0.98, 0.45))
    white_plastic = model.material("white_plastic", rgba=(0.96, 0.96, 0.97, 1.0))
    blue_plastic = model.material("blue_plastic", rgba=(0.30, 0.48, 0.78, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.16, 0.18, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _annular_tube_mesh(
            "barrel_tube",
            outer_radius=BARREL_OUTER_RADIUS,
            inner_radius=BARREL_INNER_RADIUS,
            length=BARREL_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, BARREL_START_Z)),
        material=clear_plastic,
        name="barrel_tube",
    )
    barrel.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, -0.022),
                    (0.0010, -0.020),
                    (0.0019, -0.017),
                    (0.0029, -0.012),
                    (0.0038, -0.008),
                    (0.0048, -0.003),
                    (0.0052, 0.001),
                    (0.0068, 0.005),
                    (0.0088, 0.009),
                    (0.0103, 0.012),
                    (0.0, 0.012),
                ],
                segments=64,
            ),
            "nozzle_tip",
        ),
        material=white_plastic,
        name="nozzle_tip",
    )
    barrel.visual(
        _annular_tube_mesh(
            "rear_lip",
            outer_radius=0.0122,
            inner_radius=BARREL_INNER_RADIUS,
            length=REAR_LIP_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, BARREL_START_Z + BARREL_LENGTH)),
        material=white_plastic,
        name="rear_lip",
    )
    barrel.visual(
        _annular_tube_mesh(
            "guide_ring",
            outer_radius=0.0054,
            inner_radius=0.0028,
            length=GUIDE_RING_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_RING_CENTER_Z - GUIDE_RING_LENGTH * 0.5)),
        material=white_plastic,
        name="guide_ring",
    )
    barrel.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.026, 0.012, 0.0035), 0.003),
            "left_finger_flange",
        ),
        origin=Origin(xyz=(0.019, 0.0, 0.1015)),
        material=white_plastic,
        name="left_finger_flange",
    )
    barrel.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.026, 0.012, 0.0035), 0.003),
            "right_finger_flange",
        ),
        origin=Origin(xyz=(-0.019, 0.0, 0.1015)),
        material=white_plastic,
        name="right_finger_flange",
    )
    for index, x_center in enumerate((-0.0066, 0.0066), start=1):
        barrel.visual(
            mesh_from_geometry(
                ExtrudeGeometry(rounded_rect_profile(0.0024, 0.0086, 0.0008), 0.014),
                f"end_frame_rib_{index}",
            ),
            origin=Origin(xyz=(x_center, 0.0, 0.110)),
            material=white_plastic,
            name=f"end_frame_rib_{index}",
        )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.138),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0022, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=white_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0078, length=0.0090),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=dark_rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0082, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.094)),
        material=dark_rubber,
        name="plunger_head_wiper_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0082, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=dark_rubber,
        name="plunger_head_wiper_front",
    )
    plunger.visual(
        Cylinder(radius=0.0042, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.0656)),
        material=blue_plastic,
        name="stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=blue_plastic,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=blue_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.122),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.25,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem="plunger_head",
        outer_elem="barrel_tube",
        margin=0.0027,
        name="plunger head stays centered in the barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="z",
        elem_a="plunger_head",
        elem_b="barrel_tube",
        min_overlap=0.008,
        name="plunger head remains inside barrel at rest",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="plunger_head",
            outer_elem="barrel_tube",
            margin=0.0027,
            name="plunger head stays centered when withdrawn",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="plunger_head",
            elem_b="barrel_tube",
            min_overlap=0.008,
            name="plunger head remains retained at full withdrawal",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="z",
            positive_elem="guide_ring",
            negative_elem="stop_collar",
            max_gap=0.001,
            max_penetration=0.0,
            name="end frame stop meets collar at max travel",
        )
        withdrawn_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger withdraws upward along barrel axis",
        rest_position is not None
        and withdrawn_position is not None
        and withdrawn_position[2] > rest_position[2] + 0.045,
        details=f"rest={rest_position}, withdrawn={withdrawn_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
