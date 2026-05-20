from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_cantilever_tooling_arm")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.19, 0.22, 1.0))
    dark_cover = model.material("dark_link_cover", rgba=(0.04, 0.07, 0.09, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    face_steel = model.material("plain_faceplate_steel", rgba=(0.72, 0.73, 0.71, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.52, 0.44, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="floor_plate",
    )
    tower.visual(
        Box((0.34, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=painted_steel,
        name="pedestal_block",
    )
    tower.visual(
        Box((0.20, 0.20, 0.54)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=painted_steel,
        name="tower_column",
    )
    tower.visual(
        Box((0.34, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=painted_steel,
        name="top_saddle",
    )
    tower.visual(
        Cylinder(radius=0.170, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=bearing_steel,
        name="shoulder_bearing",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.165, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=bearing_steel,
        name="proximal_bearing",
    )
    upper_link.visual(
        Box((0.82, 0.140, 0.075)),
        origin=Origin(xyz=(0.450, 0.0, 0.040)),
        material=painted_steel,
        name="main_beam",
    )
    upper_link.visual(
        Box((0.48, 0.080, 0.026)),
        origin=Origin(xyz=(0.430, 0.0, 0.087)),
        material=dark_cover,
        name="top_cover",
    )
    upper_link.visual(
        Cylinder(radius=0.145, length=0.080),
        origin=Origin(xyz=(0.900, 0.0, 0.040)),
        material=bearing_steel,
        name="distal_bearing",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.135, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=bearing_steel,
        name="proximal_bearing",
    )
    forelink.visual(
        Box((0.52, 0.120, 0.070)),
        origin=Origin(xyz=(0.280, 0.0, 0.0375)),
        material=painted_steel,
        name="main_beam",
    )
    forelink.visual(
        Box((0.34, 0.066, 0.022)),
        origin=Origin(xyz=(0.290, 0.0, 0.081)),
        material=dark_cover,
        name="top_cover",
    )
    forelink.visual(
        Cylinder(radius=0.115, length=0.075),
        origin=Origin(xyz=(0.560, 0.0, 0.0375)),
        material=bearing_steel,
        name="distal_bearing",
    )

    wrist_cartridge = model.part("wrist_cartridge")
    wrist_cartridge.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bearing_steel,
        name="wrist_hub",
    )
    wrist_cartridge.visual(
        Box((0.300, 0.085, 0.110)),
        origin=Origin(xyz=(0.160, 0.0, 0.055)),
        material=painted_steel,
        name="cartridge_body",
    )
    wrist_cartridge.visual(
        Box((0.075, 0.062, 0.055)),
        origin=Origin(xyz=(0.292, 0.0, 0.055)),
        material=dark_cover,
        name="nose_block",
    )
    wrist_cartridge.visual(
        Box((0.032, 0.180, 0.160)),
        origin=Origin(xyz=(0.340, 0.0, 0.070)),
        material=face_steel,
        name="faceplate",
    )

    model.articulation(
        "tower_to_upper",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.2, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "upper_to_forelink",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(0.900, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.5, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "forelink_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_cartridge,
        origin=Origin(xyz=(0.560, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-3.14159, upper=3.14159),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    wrist_cartridge = object_model.get_part("wrist_cartridge")

    shoulder = object_model.get_articulation("tower_to_upper")
    elbow = object_model.get_articulation("upper_to_forelink")
    wrist = object_model.get_articulation("forelink_to_wrist")

    joints = (shoulder, elbow, wrist)
    serial_chain_ok = (
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and shoulder.parent == "tower"
        and shoulder.child == "upper_link"
        and elbow.parent == "upper_link"
        and elbow.child == "forelink"
        and wrist.parent == "forelink"
        and wrist.child == "wrist_cartridge"
    )
    ctx.check("three serial revolute joints", serial_chain_ok)
    ctx.check(
        "all joint axes are vertical",
        all(abs(j.axis[0]) < 1e-6 and abs(j.axis[1]) < 1e-6 and abs(j.axis[2] - 1.0) < 1e-6 for j in joints),
    )

    ctx.expect_contact(
        tower,
        upper_link,
        elem_a="shoulder_bearing",
        elem_b="proximal_bearing",
        contact_tol=1e-5,
        name="shoulder bearing stack is seated",
    )
    ctx.expect_contact(
        upper_link,
        forelink,
        elem_a="distal_bearing",
        elem_b="proximal_bearing",
        contact_tol=1e-5,
        name="elbow bearing stack is seated",
    )
    ctx.expect_contact(
        forelink,
        wrist_cartridge,
        elem_a="distal_bearing",
        elem_b="wrist_hub",
        contact_tol=1e-5,
        name="wrist bearing stack is seated",
    )

    def x_extent(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return upper[0] - lower[0]

    upper_span = x_extent(upper_link, "main_beam")
    fore_span = x_extent(forelink, "main_beam")
    wrist_span = x_extent(wrist_cartridge, "cartridge_body")
    ctx.check(
        "links step down in length",
        upper_span is not None
        and fore_span is not None
        and wrist_span is not None
        and upper_span > fore_span + 0.20
        and fore_span > wrist_span + 0.15,
        details=f"upper={upper_span}, fore={fore_span}, wrist={wrist_span}",
    )

    faceplate_bounds = ctx.part_element_world_aabb(wrist_cartridge, elem="faceplate")
    body_bounds = ctx.part_element_world_aabb(wrist_cartridge, elem="cartridge_body")
    faceplate_ok = False
    if faceplate_bounds is not None and body_bounds is not None:
        face_min, face_max = faceplate_bounds
        body_min, body_max = body_bounds
        faceplate_ok = (
            face_max[0] > body_max[0] + 0.025
            and (face_max[1] - face_min[1]) > (body_max[1] - body_min[1]) * 1.8
            and (face_max[2] - face_min[2]) > (body_max[2] - body_min[2]) * 1.3
        )
    ctx.check("plain faceplate caps the wrist cartridge", faceplate_ok)

    rest_wrist = ctx.part_world_position(wrist_cartridge)
    with ctx.pose({shoulder: 0.45, elbow: -0.85, wrist: 1.1}):
        ctx.expect_contact(
            upper_link,
            forelink,
            elem_a="distal_bearing",
            elem_b="proximal_bearing",
            contact_tol=1e-5,
            name="elbow remains seated while swung",
        )
        ctx.expect_contact(
            forelink,
            wrist_cartridge,
            elem_a="distal_bearing",
            elem_b="wrist_hub",
            contact_tol=1e-5,
            name="wrist remains seated while swung",
        )
        swung_wrist = ctx.part_world_position(wrist_cartridge)

    ctx.check(
        "serial joints swing the wrist off the tower centerline",
        rest_wrist is not None and swung_wrist is not None and abs(swung_wrist[1] - rest_wrist[1]) > 0.15,
        details=f"rest={rest_wrist}, swung={swung_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
