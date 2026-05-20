from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    radial_segments: int = 72,
) -> MeshGeometry:
    return boolean_difference(
        CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments),
        CylinderGeometry(
            radius=inner_radius,
            height=length + 0.002,
            radial_segments=radial_segments,
        ),
    )


def _barrel_body_mesh() -> MeshGeometry:
    body = MeshGeometry()
    body.merge(_ring_shell(0.0023, 0.0010, 0.014).translate(0.0, 0.0, 0.007))
    body.merge(_ring_shell(0.0062, 0.0023, 0.014).translate(0.0, 0.0, 0.015))
    body.merge(_ring_shell(0.0115, 0.0062, 0.006).translate(0.0, 0.0, 0.023))
    body.merge(_ring_shell(0.0115, 0.0094, 0.078).translate(0.0, 0.0, 0.065))
    body.merge(_ring_shell(0.0130, 0.0102, 0.010).translate(0.0, 0.0, 0.109))

    taper = boolean_difference(
        ConeGeometry(radius=0.0056, height=0.010, radial_segments=72, closed=True),
        ConeGeometry(radius=0.0040, height=0.012, radial_segments=72, closed=True),
    ).translate(0.0, 0.0, 0.017)
    body.merge(taper)
    return body


def _guide_cage_mesh() -> MeshGeometry:
    cage = MeshGeometry()
    cage.merge(_ring_shell(0.0115, 0.0052, 0.008).translate(0.0, 0.0, 0.116))
    cage.merge(_ring_shell(0.0115, 0.0052, 0.009).translate(0.0, 0.0, 0.158))
    for x, y in ((0.0080, 0.0), (-0.0080, 0.0), (0.0, 0.0080), (0.0, -0.0080)):
        cage.merge(CylinderGeometry(radius=0.0016, height=0.048, radial_segments=24).translate(x, y, 0.137))
    return cage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.84, 0.93, 0.98, 0.38))
    barrel_blue = model.material("barrel_blue", rgba=(0.66, 0.84, 0.94, 0.60))
    plunger_white = model.material("plunger_white", rgba=(0.96, 0.96, 0.97, 1.0))
    stopper_black = model.material("stopper_black", rgba=(0.14, 0.15, 0.16, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.42, 0.62, 0.82, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("syringe_barrel_body", _barrel_body_mesh()),
        material=clear_poly,
        name="barrel_body",
    )
    barrel.visual(
        _save_mesh("syringe_guide_cage", _guide_cage_mesh()),
        material=barrel_blue,
        name="guide_cage",
    )
    barrel.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(xyz=(0.020, 0.0, 0.109)),
        material=grip_blue,
        name="right_finger_grip",
    )
    barrel.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(xyz=(-0.020, 0.0, 0.109)),
        material=grip_blue,
        name="left_finger_grip",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.162)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0089, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=stopper_black,
        name="head_front",
    )
    plunger.visual(
        Cylinder(radius=0.0086, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=stopper_black,
        name="head_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=stopper_black,
        name="head_core",
    )
    plunger.visual(
        Cylinder(radius=0.0021, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=plunger_white,
        name="rod",
    )
    plunger.visual(
        Box((0.0042, 0.0070, 0.058)),
        origin=Origin(xyz=(0.0040, 0.0, 0.030)),
        material=plunger_white,
        name="right_guide_fin",
    )
    plunger.visual(
        Box((0.0042, 0.0070, 0.058)),
        origin=Origin(xyz=(-0.0040, 0.0, 0.030)),
        material=plunger_white,
        name="left_guide_fin",
    )
    plunger.visual(
        Cylinder(radius=0.0046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=plunger_white,
        name="thumb_post",
    )
    plunger.visual(
        Cylinder(radius=0.0165, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=plunger_white,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.034, 0.018, 0.178)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.064,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.expect_overlap(
        plunger,
        barrel,
        axes="xy",
        elem_a="head_front",
        elem_b="barrel_body",
        min_overlap=0.016,
        name="plunger head sits inside the barrel bore at rest",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="z",
        elem_a="rod",
        elem_b="guide_cage",
        min_overlap=0.040,
        name="rod visibly runs through the rear guide cage at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="xy",
            elem_a="head_front",
            elem_b="barrel_body",
            min_overlap=0.016,
            name="plunger head stays centered when retracted",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="rod",
            elem_b="guide_cage",
            min_overlap=0.040,
            name="rod remains visibly engaged with the guide cage at full retraction",
        )
        retract_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts rearward along the barrel axis",
        rest_pos is not None and retract_pos is not None and retract_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, retracted={retract_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
