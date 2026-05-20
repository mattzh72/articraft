import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_spindle")

    # 1. Rail
    rail = model.part("rail")
    rail_length = 1.0
    rail_width = 0.05
    rail_height = 0.02
    
    rail_cq = cq.Workplane("XY").box(rail_length, rail_width, rail_height)
    rail_cq = rail_cq.translate((0, 0, rail_height / 2))
    rail.visual(mesh_from_cadquery(rail_cq, "rail"), name="rail_body")

    # 2. Carriage
    carriage = model.part("carriage")
    c_len = 0.1
    c_width = 0.072
    
    # Top plate sits exactly on the rail (Z=0.020 to Z=0.040)
    top_plate_height = 0.020
    top_plate_z = 0.020 + top_plate_height / 2
    car_cq = cq.Workplane("XY").box(c_len, c_width, top_plate_height).translate((0, 0, top_plate_z))
    
    # Side flanges go down along the rail (Z=0.005 to Z=0.025 to overlap top plate)
    flange_height = 0.020
    flange_z = 0.005 + flange_height / 2
    flange_width = (c_width - 0.05) / 2  # exactly 0.011
    flange_y_offset = 0.025 + flange_width / 2
    
    flange_left = cq.Workplane("XY").box(c_len, flange_width, flange_height).translate((0, flange_y_offset, flange_z))
    flange_right = cq.Workplane("XY").box(c_len, flange_width, flange_height).translate((0, -flange_y_offset, flange_z))
    
    car_cq = car_cq.union(flange_left).union(flange_right)
    
    # Bearing housing
    bh_len = 0.06
    bh_width = 0.05
    bh_height = 0.065
    bh_center_z = 0.035 + bh_height / 2  # starts from Z=0.035 to Z=0.100 to overlap top plate
    spindle_z = 0.07
    bh_cq = cq.Workplane("XY").box(bh_len, bh_width, bh_height).translate((0, 0, bh_center_z))
    car_cq = car_cq.union(bh_cq)
    
    # Bore for the spindle
    bore_radius = 0.016
    bore = cq.Workplane("XZ").workplane(offset=-bh_width / 2 - 0.01).center(0, spindle_z).circle(bore_radius).extrude(bh_width + 0.02)
    car_cq = car_cq.cut(bore)
    
    carriage.visual(mesh_from_cadquery(car_cq, "carriage"), name="carriage_body")
    
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.4, upper=0.4, effort=100.0, velocity=1.0),
    )

    # 3. Spindle
    spindle = model.part("spindle")
    spindle_r = 0.015
    chuck_r = 0.025
    chuck_len = 0.03
    
    # Shaft fits inside the bore (Y from -0.025 to 0.025)
    # Extrude slightly longer to ensure robust union with the chuck
    shaft = cq.Workplane("XZ").workplane(offset=-bh_width / 2).circle(spindle_r).extrude(bh_width + 0.01)
    # Chuck extends from the +Y face of the bearing housing (Y from 0.025 to 0.055)
    chuck = cq.Workplane("XZ").workplane(offset=bh_width / 2).circle(chuck_r).extrude(chuck_len)
    
    # Cut a small hole in the chuck to make it look like a tool holder
    tool_hole = cq.Workplane("XZ").workplane(offset=bh_width / 2 + chuck_len - 0.015).circle(0.008).extrude(0.015)
    
    spindle_cq = shaft.union(chuck).cut(tool_hole)
    
    spindle.visual(mesh_from_cadquery(spindle_cq, "spindle"), name="spindle_body")
    
    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, spindle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    
    # The spindle shaft is captured inside the bearing housing bore
    ctx.allow_overlap(
        carriage,
        spindle,
        reason="Spindle shaft is captured inside the carriage bearing housing.",
        elem_a="carriage_body",
        elem_b="spindle_body",
    )
    
    # The carriage wraps the rail for a sliding proxy fit
    ctx.allow_overlap(
        carriage,
        rail,
        reason="Carriage wraps the rail in a sliding proxy fit.",
        elem_a="carriage_body",
        elem_b="rail_body",
    )
    
    # Proof of correct carriage mounting
    ctx.expect_contact(carriage, rail, name="carriage rests on rail")
    ctx.expect_within(carriage, rail, axes="y", margin=0.02, name="carriage stays centered on rail")
    ctx.expect_overlap(carriage, rail, axes="x", min_overlap=0.08, name="carriage remains on rail")
    
    # Proof of correct spindle mounting
    ctx.expect_within(spindle, carriage, axes="xz", margin=0.0, name="spindle stays within bearing housing footprint")
    
    # Check carriage motion
    with ctx.pose({"carriage_slide": 0.3}):
        ctx.expect_overlap(carriage, rail, axes="x", min_overlap=0.08, name="carriage remains on rail at extension")

    return ctx.report()


object_model = build_object_model()
